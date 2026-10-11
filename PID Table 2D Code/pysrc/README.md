# Visualisation de la cinématique inverse

Python 3.10 ou supérieur. Depuis le dossier `PID Table 2D Code` :

```powershell
python -m pip install -r pysrc/requirements.txt
python pysrc/kinematics_viewer.py
```

Les dimensions et limites sont **lues directement dans** `src/kinematics.h` : aucun deuxième jeu de paramètres à synchroniser. Les initialisations doivent être des nombres, pas des expressions C++. Le bouton **Relire les paramètres** recharge le fichier après modification.

Les curseurs donnent z en mm au-dessus de h0, roll et pitch en degrés. La figure montre les axes servos S1–S3, les extrémités des palonniers E1–E3, les rotules A/B/C et le centre P. Les axes colorés sur P représentent le repère de la plateforme. Les plans radiaux sont optionnels. La souris permet de tourner et zoomer la vue ; la caméra reste à sa position quand les curseurs changent.

Les calculs reprennent `kinematics.cpp` : ordre Rz(yaw) Ry(pitch) Rx(roll), compensations xP/yP, branche yaw proche de zéro, branches de bras fixes, tolérances et limites angulaires identiques. q=0 pointe horizontalement vers l'extérieur ; q positif monte le palonnier. Les diagnostics indiquent les erreurs de fermeture des bielles et d'appartenance aux plans, en mm.

Une cible impossible est dessinée en rouge, avec la dernière structure valide en transparence. Aucun angle calculé partiellement n'est présenté comme une solution. Les trois curseurs commencent à zéro. Plages personnalisables :

```powershell
python pysrc/kinematics_viewer.py --z-max 70 --tilt-max 30
```

Le modèle ne vérifie pas les collisions ni les limites d'articulation des rotules. Il n'envoie aucune commande au matériel. L'échelle de la vue est identique dans les trois directions et reste fixe pendant la manipulation.

Tests :

```powershell
python -m unittest discover -s pysrc -p 'test_*.py' -v
```

La comparaison directe au C++ est optionnelle. Compiler `reference_bridge.cpp` avec le vrai `kinematics.cpp`, puis définir `KINEMATICS_REFERENCE` vers l'exécutable avant de lancer les tests. Par exemple avec g++ :

```powershell
g++ -std=c++17 src/kinematics.cpp pysrc/reference_bridge.cpp -o pysrc/reference_bridge.exe
$env:KINEMATICS_REFERENCE = (Resolve-Path pysrc/reference_bridge.exe).Path
python -m unittest discover -s pysrc -p 'test_*.py' -v
```

Cette comparaison porte sur 1 160 poses, y compris des poses rejetées ; elle vérifie les angles, xP, yP, le yaw, le type d'erreur et le bras concerné.

Une image de contrôle peut être produite sans fenêtre :

```powershell
$env:MPLBACKEND = 'Agg'
python pysrc/kinematics_viewer.py --snapshot pysrc/preview.png
```
