"""Run: python pysrc/kinematics_viewer.py (Python 3.10+)."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from kinematics_model import DEFAULT_HEADER, load_parameters, compute_kinematics

COLORS = ('#0077bb', '#ee7733', '#009988')
ERRORS = {'InvalidParameters': 'Paramètres invalides', 'InvalidInput': 'Consigne invalide',
          'SingularOrientation': 'Orientation singulière', 'SingularLeg': 'Bras singulier',
          'Unreachable': 'Position hors de portée', 'JointLimit': 'Limite angulaire dépassée'}


class Viewer:
    def __init__(self, header, z_max=None, tilt_max=40):
        self.header = Path(header)
        self.parameters = load_parameters(self.header)
        self.last_valid = None
        self.show_planes = False
        self.fig = plt.figure(figsize=(13, 9))
        self.fig.canvas.manager.set_window_title('PID Table — debug cinématique inverse')
        self.ax = self.fig.add_axes([.03, .28, .68, .66], projection='3d')
        self.ax.view_init(elev=25, azim=-55)
        self.fig.text(.04, .96, 'Cinématique inverse — plateforme 3 bras', fontsize=17, weight='bold')
        self.info = self.fig.text(.73, .88, '', va='top', family='monospace', fontsize=10)
        self.status = self.fig.text(.04, .245, '', fontsize=11)
        self.feedback = self.fig.text(.73, .32, '', fontsize=9, wrap=True)
        z_max = z_max if z_max is not None else max(50, self.parameters.L1+self.parameters.L2-self.parameters.h0)
        self.sliders = [
            Slider(self.fig.add_axes([.15, .17, .52, .025]), 'z (mm)', 0, z_max, valinit=0),
            Slider(self.fig.add_axes([.15, .12, .52, .025]), 'roll (°)', -tilt_max, tilt_max, valinit=0),
            Slider(self.fig.add_axes([.15, .07, .52, .025]), 'pitch (°)', -tilt_max, tilt_max, valinit=0)]
        self.reset = Button(self.fig.add_axes([.74, .10, .20, .045]), 'Pose initiale')
        self.reload = Button(self.fig.add_axes([.74, .16, .20, .045]), 'Relire les paramètres')
        self.check = CheckButtons(self.fig.add_axes([.73, .23, .23, .06]), ['Plans radiaux'], [False])
        self.reset.on_clicked(self.reset_pose)
        self.reload.on_clicked(self.reload_parameters)
        self.check.on_clicked(self.toggle_planes)
        for slider in self.sliders:
            slider.on_changed(self.update)
        self.update()

    def reset_pose(self, _):
        for slider in self.sliders:
            slider.reset()

    def reload_parameters(self, _):
        try:
            candidate = load_parameters(self.header)
            if compute_kinematics(candidate, 0, 0, 0).error == 'InvalidParameters':
                raise ValueError('Dimensions ou limites invalides')
            self.parameters = candidate
            self.last_valid = None
            self.feedback.set_text('Paramètres relus depuis kinematics.h')
        except (OSError, ValueError) as exc:
            self.feedback.set_text(f'Lecture impossible : {exc}')
        self.update()

    def toggle_planes(self, _):
        self.show_planes = not self.show_planes
        self.update()

    def plot_line(self, points, **kwargs):
        self.ax.plot(*np.asarray(points).T, **kwargs)

    def draw_structure(self, result, ghost=False):
        alpha = .3 if ghost else 1
        for i, color in enumerate(COLORS):
            base, elbow, joint = result.bases[i], result.elbows[i], result.joints[i]
            self.plot_line([base, elbow], color=color, linewidth=5, alpha=alpha)
            self.plot_line([elbow, joint], color=color, linewidth=2.5, alpha=alpha)
            self.ax.scatter(*np.array([base, elbow, joint]).T, color=color, s=35, alpha=alpha)
            for name, point in zip((f'S{i+1}', f'E{i+1}', 'ABC'[i]), (base, elbow, joint)):
                self.ax.text(*point, ' '+name, color=color, alpha=alpha)
        self.ax.add_collection3d(Poly3DCollection([result.joints], facecolors='#66aadd',
                                                  edgecolors='#335577', alpha=.08 if ghost else .22))
        self.ax.scatter(*result.center, color='#222222', s=35, alpha=alpha)
        self.ax.text(*result.center, ' P', alpha=alpha)
        for i, color in enumerate(('red', 'green', 'blue')):
            end = result.center+result.rotation[:, i]*30
            self.plot_line([result.center, end], color=color, alpha=alpha)
            self.ax.text(*end, 'xyz'[i], color=color, alpha=alpha)

    def update(self, _=None):
        z, roll, pitch = (slider.val for slider in self.sliders)
        p = self.parameters
        result = compute_kinematics(p, z, roll, pitch)
        elev, azim = self.ax.elev, self.ax.azim
        self.ax.clear()
        self.ax.view_init(elev=elev, azim=azim)
        radius = max(p.Rb+p.L1, 1.6*p.Rp, 50)
        top = max(p.L1+p.L2, p.h0+self.sliders[0].valmax+p.Rp, 100)
        self.ax.set(xlim=(-radius, radius), ylim=(-radius, radius), zlim=(-p.L1, top),
                    xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)')
        self.ax.set_box_aspect((2*radius, 2*radius, top+p.L1))
        theta = np.linspace(0, 2*np.pi, 100)
        self.plot_line(np.column_stack((p.Rb*np.cos(theta), p.Rb*np.sin(theta), np.zeros_like(theta))),
                       color='gray', linestyle='--', linewidth=1)
        if self.show_planes:
            for i, color in enumerate(COLORS):
                e = np.array([np.cos(i*2*np.pi/3), np.sin(i*2*np.pi/3), 0])
                corners = [-radius*e, radius*e, radius*e+[0, 0, top], -radius*e+[0, 0, top]]
                self.ax.add_collection3d(Poly3DCollection([corners], facecolors=color, alpha=.06))
        if result.valid:
            self.last_valid = result
            self.draw_structure(result)
            self.status.set_text('Pose valide — L1 : traits épais, L2 : traits fins. Souris : rotation/zoom de la vue.')
            self.status.set_color('#007744')
        else:
            if self.last_valid is not None:
                self.draw_structure(self.last_valid, ghost=True)
            if result.joints is not None:
                target = np.vstack((result.joints, result.joints[0]))
                self.plot_line(target, color='crimson', linestyle='--', linewidth=2)
                self.ax.scatter(*result.joints.T, color='crimson', marker='x', s=65)
            leg = '' if result.failed_leg < 0 else f' — bras {"ABC"[result.failed_leg]}'
            self.status.set_text(ERRORS[result.error]+leg+' | cible rouge ; dernière pose valide en transparence')
            self.status.set_color('crimson')
        text = (f'PARAMÈTRES (mm)\nL1 = {p.L1:g}\nL2 = {p.L2:g}\nRb = {p.Rb:g}\nRp = {p.Rp:g}\nh0 = {p.h0:g}\n'
                f'\nCONSIGNE\nz     = {z:8.3f} mm\nroll  = {roll:8.3f} °\npitch = {pitch:8.3f} °\n')
        if result.center is not None:
            text += (f'\nCOMPENSATIONS\nxP  = {result.center[0]:8.3f} mm\nyP  = {result.center[1]:8.3f} mm\n'
                     f'zP  = {result.center[2]:8.3f} mm\nyaw = {result.yaw_deg:8.3f} °\n')
        if result.valid:
            text += '\nANGLES SERVOS (°)\n'
            for i, q in enumerate(result.angles):
                text += f'q{i+1} = {q:8.3f}\n'
            length_error = np.max(np.abs(np.linalg.norm(result.joints-result.elbows, axis=1)-p.L2))
            alpha = np.arange(3)*2*np.pi/3
            plane_error = np.max(np.abs(-np.sin(alpha)*result.joints[:, 0]+np.cos(alpha)*result.joints[:, 1]))
            text += f'\nERREURS GÉOMÉTRIQUES\nL2    : {length_error:.2e} mm\nPlans : {plane_error:.2e} mm\n'
        else:
            text += f'\nPAS DE SOLUTION\n{result.error}\nAngles non appliqués\n'
        text += '\nLimites q (°) :\n'+ '\n'.join(f'{i+1}: [{p.q_min[i]:g}, {p.q_max[i]:g}]  branche {p.elbow_sign[i]:g}' for i in range(3))
        self.info.set_text(text)
        self.fig.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--header', type=Path, default=DEFAULT_HEADER)
    parser.add_argument('--z-max', type=float)
    parser.add_argument('--tilt-max', type=float, default=40)
    parser.add_argument('--snapshot', type=Path, help='Enregistre une image sans ouvrir de fenêtre')
    args = parser.parse_args()
    if (args.z_max is not None and args.z_max <= 0) or args.tilt_max <= 0:
        parser.error('Les amplitudes des curseurs doivent être positives')
    try:
        viewer = Viewer(args.header, args.z_max, args.tilt_max)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'Erreur de paramètres : {exc}\n')
    if args.snapshot:
        viewer.fig.savefig(args.snapshot, dpi=140)
        plt.close(viewer.fig)
    else:
        plt.show()


if __name__ == '__main__':
    main()
