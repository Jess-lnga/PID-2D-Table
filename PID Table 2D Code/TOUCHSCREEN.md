# Écran tactile résistif quatre fils

Le module `src/tch_screen.h/.cpp` lit la position indépendamment du mode IMU.
Il est initialisé dans `setup()` et actualisé dans la boucle sous `RuntimeLock`.
Il ne commande pas les servos : la position est pour le moment une mesure disponible
via `TchScreen::getState()` et l'interface web.

## Câblage

| Électrode | GPIO ESP32 |
| --- | --- |
| P1 | 33 |
| P2 | 13 **et 34** (relier les deux à la même électrode) |
| P3 | 23 |
| P4 | 14 |

Les paires résistives sont P1–P3 et P2–P4. GPIO34 est uniquement une entrée
de mesure reliée à P2 : GPIO13 continue à exciter la couche. Son ADC2
n'est pas disponible lorsque le Wi-Fi fonctionne. P1 est lu directement sur GPIO33.
Les mesures passent par ADC1 (GPIO33 et GPIO34), compatible avec le Wi-Fi.
Les quatre électrodes sont commandées par le module ; ne pas les raccorder
à une alimentation externe. Les signaux restent au niveau logique 3,3 V.

## Lecture et affichage

Le module attend au moins 1 ms pour les axes et 5 ms pour chaque étape de
détection, sans appel bloquant à `delay`. Les cycles sont espacés d'au moins
10 ms ; un cycle avec contact nécessite au moins 22 ms, plus le temps de boucle
et de conversion. Chaque axe est filtré par une médiane de cinq conversions.
La détection de contact excite une couche successivement au niveau bas puis haut,
l'autre restant en entrée avec résistance de rappel interne au niveau bas.
GPIO13 et GPIO34 doivent tous deux suivre la séquence 0 puis 1, avant et après
les mesures des axes. Un niveau bloqué à 1 est rejeté.
Deux cycles consécutifs de contact sont requis pour acquérir la balle. Ensuite,
la présence est conservée jusqu'à 50 ms après la dernière position valide
(`CONTACT_HOLD_MS` dans `tch_screen.h`). Un cycle valide dans ce délai reprend
immédiatement le suivi. Après expiration, deux cycles sont de nouveau nécessaires.
Les électrodes sont remises en entrée au repos.

`State` fournit `present`, `rawX`, `rawY`, `x`, `y`, `samples` et `lastUpdateMs`.
`x` et `y` sont compris entre 0 et 1. Les coordonnées précédentes restent en mémoire
en l'absence de contact : toujours vérifier `present` et l'âge de la mesure.
`measurementValid` indique si le dernier cycle a fourni une position acceptée ;
`validSamples` compte ces positions et `lastValidMs` date la dernière. Ces champs
ne sont pas rafraîchis par un cycle raté, sauf `measurementValid` qui devient faux.
Pour un contrôleur, consommer les nouvelles positions via `validSamples` et vérifier
leur âge ; une présence maintenue ne constitue pas une nouvelle mesure.
Le JSON `/state` ajoute l'objet `touch` avec ces coordonnées, `ageMs` (dernier cycle),
`measurementValid`, `validSamples` et `positionAgeMs` (dernière position, `null`
avant la première). Pendant une coupure, le point devient translucide et le statut
signale la conservation de la dernière position ; aucune extrapolation n'est faite.
La page interroge l'état environ toutes les 100 ms (200 ms pendant la calibration
servo, plus le temps de réponse) et masque la balle en cas d'absence de contact,
de données trop anciennes ou d'erreur réseau.

## Mise en service

1. Vérifier le câblage supplémentaire P2–GPIO34, puis flasher le firmware.
2. Ouvrir l'interface web habituelle : la section « Position de la balle » doit
   indiquer l'absence de balle lorsque la surface est libre.
3. Appuyer au centre et aux quatre coins ; vérifier les coordonnées ADC et le
   déplacement du point. X correspond à la couche 2–4, Y à la couche 1–3.
4. Relever les valeurs aux limites utiles, puis ajuster `X_MIN`, `X_MAX`, `Y_MIN`,
   `Y_MAX` dans `tch_screen.h` et recompiler. `X_MIN`/`X_MAX` sont les lectures
   2–4 aux bords gauche/droit ; `Y_MIN`/`Y_MAX` sont les lectures 1–3 aux bords
   haut/bas. Pour inverser un axe, inverser ses
   deux limites (elles doivent être distinctes). Par défaut l'étendue est 0–4095 :
   les pourcentages sont indicatifs et ne représentent pas des millimètres.
5. Tester avec la balle réelle : une dalle résistive nécessite une pression
   suffisante. Un objet posé sans fermeture électrique ne sera pas détecté.
   La détection signale tout contact, sans pouvoir distinguer une balle d'un doigt.

La vue respecte le rectangle de 26 cm en X (horizontal) sur 19,8 cm en Y
(vertical). Ce rapport d'affichage ne remplace pas la calibration ADC aux bords.

## Pilotage sans IMU

Quand l'interrupteur IMU (GPIO32) est désactivé, le menu « Pilotage sans IMU »
propose la vague existante (roll/pitch ±5°, période 4,5 s) ou le mode manuel.
La vague reste le mode par défaut au démarrage.

En manuel, les sliders pitch et roll vont de −10° à +10°, par pas de 0,1°.
Une commande est envoyée au relâchement du slider. « Remettre à plat » demande
roll = pitch = 0 à la hauteur courante. Les poses passent par la cinématique
inverse et les calibrations des trois servos : une pose impossible, hors plage
ou une calibration manquante fait refuser la commande sans changer les consignes.
Le changement de hauteur tient compte des angles manuels sélectionnés.

L'IMU et le menu de calibration des servos restent prioritaires et bloquent les
commandes web de ce menu, y compris côté serveur. Le choix hors IMU et les angles
sont conservés en RAM pendant cette interruption et repris à sa fin ; ils ne sont
pas enregistrés en mémoire permanente. Une perte de connexion web laisse le mode
et la dernière consigne acceptée actifs sur l'ESP32 (la vague continue, le manuel
maintient sa pose). Les commandes manuelles sont appliquées dans la boucle de
contrôle, pas dans la tâche HTTP.

## Diagnostic de faux contact

La page affiche « Test contact LOW / HIGH » pour GPIO13 et GPIO34, même sans
contact. Il s'agit des niveaux numériques du dernier test précédant les axes.
Après flash et rechargement de la page, relever les deux paires :

- Dalle branchée sans pression : 0 / 0 attendu sur les deux GPIO.
- En appuyant au centre : 0 / 1 attendu sur les deux GPIO.
- Après relâchement : retour à 0 / 0 attendu.

Les deux GPIO sont reliés par le pont : des résultats différents suggèrent un
problème de connexion, une transition ou une configuration à examiner. Un
résultat 0 / 1 sans pression reste un faux contact à investiguer ; ce diagnostic
ne prouve pas à lui seul que la dalle est défectueuse.
