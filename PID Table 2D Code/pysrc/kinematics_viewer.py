"""Run: python pysrc/kinematics_viewer.py (Python 3.10+)."""
import argparse
import time
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
    def __init__(self, header, z_max=None, tilt_max=40, fps=30, use_blit=True):
        self.header = Path(header)
        self.parameters = load_parameters(self.header)
        self.last_valid = None
        self.show_planes = False
        self._background = None
        self._dirty = False
        self._saving = False
        self._text_cache = {}
        self._info_last = 0.0
        self._info_kind = None
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
        self._use_blit = use_blit and self.fig.canvas.supports_blit
        self._interactive = self.fig.canvas.required_interactive_framework is not None
        self._frame_interval_ms = max(1, round(1000/fps))
        self._frame_started = 0.0
        self._timer = self.fig.canvas.new_timer(interval=self._frame_interval_ms)
        self._timer.single_shot = True
        self._timer.add_callback(self._flush_update)
        self._create_artists()
        self._configure_scene()
        for slider in self.sliders:
            # Slider redraws would otherwise trigger a second full figure render.
            slider.drawon = False
            slider.on_changed(self.request_update)
        self.fig.canvas.mpl_connect('draw_event', self._on_draw)
        self.fig.canvas.mpl_connect('resize_event', self._invalidate_background)
        self.fig.canvas.mpl_connect('button_release_event', self._flush_update)
        self.fig.canvas.mpl_connect('close_event', lambda event: self._timer.stop())
        self.update()

    def request_update(self, _=None):
        if not self._interactive:
            self.update()
            return
        was_dirty = self._dirty
        self._dirty = True
        if not was_dirty:
            elapsed_ms = (time.perf_counter()-self._frame_started)*1000
            self._timer.interval = max(1, round(self._frame_interval_ms-elapsed_ms))
            self._timer.start()

    def _flush_update(self, _=None):
        if self._dirty:
            self._timer.stop()
            self._dirty = False
            self.update(force_info=getattr(_, "name", None) == "button_release_event")
        elif getattr(_, 'name', None) == 'button_release_event':
            self.update(force_info=True)

    def _invalidate_background(self, _=None):
        self._background = None

    def _on_draw(self, event):
        if not self._use_blit or self._saving:
            return
        self._text_cache.clear()
        self._background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        self._draw_dynamic()

    def _draw_dynamic(self):
        for artist in self._dynamic_artists:
            if artist.get_visible():
                if isinstance(artist, Poly3DCollection):
                    artist.do_3d_projection()
                if artist in (self.info, self.status, self.feedback):
                    # Reuse rasterized diagnostics/status when their content is unchanged.
                    key = (artist.get_text(), str(artist.get_color()))
                    cached = self._text_cache.get(artist)
                    if cached is not None and cached[0] == key:
                        self.fig.canvas.restore_region(cached[1])
                        continue
                    self.fig.draw_artist(artist)
                    if artist.get_text():
                        bbox = artist.get_window_extent(self.fig.canvas.get_renderer()).expanded(1.02, 1.05)
                        self._text_cache[artist] = (key, self.fig.canvas.copy_from_bbox(bbox))
                else:
                    self.fig.draw_artist(artist)

    def _render(self):
        if self._use_blit and self._background is not None:
            self.fig.canvas.restore_region(self._background)
            self._draw_dynamic()
            self.fig.canvas.blit(self.fig.bbox)
        else:
            self.fig.canvas.draw_idle()

    def save_snapshot(self, path):
        self.update(force_info=True)
        # Animated artists are excluded by savefig unless explicitly restored.
        self._saving = True
        try:
            for artist in self._dynamic_artists:
                artist.set_animated(False)
            self.fig.savefig(path, dpi=140)
        finally:
            for artist in self._dynamic_artists:
                artist.set_animated(self._use_blit)
            self._saving = False
            self._background = None

    def reset_pose(self, _):
        for slider in self.sliders:
            slider.reset()
        self._timer.stop()
        self._dirty = False
        self.update(force_info=True)

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
        self._configure_scene()
        self.update()

    def toggle_planes(self, _):
        self.show_planes = not self.show_planes
        for plane in self.planes:
            plane.set_visible(self.show_planes)
        self._background = None
        self.update()

    def _create_artists(self):
        # All objects persist across poses. No ax.clear(), new labels or collections.
        self.platform = Poly3DCollection([np.zeros((3, 3))], facecolors='#66aadd', edgecolors='#335577', alpha=.22)
        self.ax.add_collection3d(self.platform)
        self.structure_artists = [self.platform]
        self.legs = []
        for i, color in enumerate(COLORS):
            upper, = self.ax.plot([], [], [], color=color, linewidth=5)
            lower, = self.ax.plot([], [], [], color=color, linewidth=2.5)
            points, = self.ax.plot([], [], [], linestyle='', marker='o', color=color, markersize=6)
            labels = [self.ax.text(0, 0, 0, ' '+name, color=color)
                      for name in (f'S{i+1}', f'E{i+1}', 'ABC'[i])]
            self.legs.append((upper, lower, points, labels))
            self.structure_artists.extend([upper, lower, points, *labels])
        self.center_marker, = self.ax.plot([], [], [], linestyle='', marker='o', color='#222222')
        self.center_label = self.ax.text(0, 0, 0, ' P')
        self.structure_artists.extend([self.center_marker, self.center_label])
        self.axes_lines = []
        for i, color in enumerate(('red', 'green', 'blue')):
            line, = self.ax.plot([], [], [], color=color)
            label = self.ax.text(0, 0, 0, 'xyz'[i], color=color)
            self.axes_lines.append((line, label))
            self.structure_artists.extend([line, label])
        self.target_line, = self.ax.plot([], [], [], color='crimson', linestyle='--', linewidth=2)
        self.target_points, = self.ax.plot([], [], [], color='crimson', linestyle='', marker='x', markersize=8)
        self.base_circle, = self.ax.plot([], [], [], color='gray', linestyle='--', linewidth=1)
        self.planes = []
        for color in COLORS:
            plane = Poly3DCollection([np.zeros((4, 3))], facecolors=color, alpha=.06, visible=False)
            self.ax.add_collection3d(plane)
            self.planes.append(plane)
        # Draw 2D slider axes as a unit, avoiding private Slider handle attributes.
        self._dynamic_artists = [*self.structure_artists, self.target_line, self.target_points,
                                 self.info, self.status, self.feedback, *(slider.ax for slider in self.sliders)]
        for artist in self._dynamic_artists:
            artist.set_animated(self._use_blit)
        for artist in self.structure_artists:
            artist.set_visible(False)
        self.target_line.set_visible(False)
        self.target_points.set_visible(False)

    @staticmethod
    def _set_line(artist, points):
        artist.set_data_3d(*np.asarray(points).T)

    @staticmethod
    def _set_label(artist, point):
        artist.set_position_3d(tuple(point))

    def _configure_scene(self):
        # Limits and background only change when dimensions are reloaded.
        p = self.parameters
        radius = max(p.Rb+p.L1, 1.6*p.Rp, 50)
        top = max(p.L1+p.L2, p.h0+self.sliders[0].valmax+p.Rp, 100)
        self.ax.set(xlim=(-radius, radius), ylim=(-radius, radius), zlim=(-p.L1, top),
                    xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)')
        self.ax.set_box_aspect((2*radius, 2*radius, top+p.L1))
        self.ax.set_autoscale_on(False)
        theta = np.linspace(0, 2*np.pi, 80)
        self._set_line(self.base_circle, np.column_stack((p.Rb*np.cos(theta), p.Rb*np.sin(theta), np.zeros_like(theta))))
        for i, plane in enumerate(self.planes):
            e = np.array([np.cos(i*2*np.pi/3), np.sin(i*2*np.pi/3), 0])
            plane.set_verts([[-radius*e, radius*e, radius*e+[0, 0, top], -radius*e+[0, 0, top]]])
            plane.set_visible(self.show_planes)
        self._background = None
        self._info_kind = None

    def draw_structure(self, result, ghost=False):
        alpha = .3 if ghost else 1
        for artist in self.structure_artists:
            artist.set_visible(True)
            artist.set_alpha(alpha)
        self.platform.set_alpha(.08 if ghost else .22)
        self.platform.set_verts([result.joints])
        for i, (upper, lower, points, labels) in enumerate(self.legs):
            base, elbow, joint = result.bases[i], result.elbows[i], result.joints[i]
            self._set_line(upper, [base, elbow])
            self._set_line(lower, [elbow, joint])
            self._set_line(points, [base, elbow, joint])
            for label, point in zip(labels, (base, elbow, joint)):
                self._set_label(label, point)
        self._set_line(self.center_marker, [result.center])
        self._set_label(self.center_label, result.center)
        for i, (line, label) in enumerate(self.axes_lines):
            end = result.center+result.rotation[:, i]*30
            self._set_line(line, [result.center, end])
            self._set_label(label, end)

    def update(self, _=None, force_info=False):
        self._frame_started = time.perf_counter()
        z, roll, pitch = (slider.val for slider in self.sliders)
        p = self.parameters
        result = compute_kinematics(p, z, roll, pitch)
        self.target_line.set_visible(False)
        self.target_points.set_visible(False)
        if result.valid:
            self.last_valid = result
            self.draw_structure(result)
            self.status.set_text('Pose valide — L1 : traits épais, L2 : traits fins. Souris : rotation/zoom de la vue.')
            self.status.set_color('#007744')
        else:
            if self.last_valid is not None:
                self.draw_structure(self.last_valid, ghost=True)
            else:
                for artist in self.structure_artists:
                    artist.set_visible(False)
            if result.joints is not None:
                target = np.vstack((result.joints, result.joints[0]))
                self._set_line(self.target_line, target)
                self._set_line(self.target_points, result.joints)
                self.target_line.set_visible(True)
                self.target_points.set_visible(True)
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
        now = time.perf_counter()
        if force_info or result.error != self._info_kind or now-self._info_last >= .12:
            self.info.set_text(text)
            self._info_last = now
            self._info_kind = result.error
        self._render()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--header', type=Path, default=DEFAULT_HEADER)
    parser.add_argument('--z-max', type=float)
    parser.add_argument('--tilt-max', type=float, default=40)
    parser.add_argument('--fps', type=int, default=30, help='Fréquence maximale des mises à jour (défaut : 30)')
    parser.add_argument('--no-blit', action='store_true', help='Désactive le rendu accéléré pour compatibilité')
    parser.add_argument('--snapshot', type=Path, help='Enregistre une image sans ouvrir de fenêtre')
    args = parser.parse_args()
    if (args.z_max is not None and args.z_max <= 0) or args.tilt_max <= 0 or args.fps <= 0:
        parser.error('Les amplitudes des curseurs doivent être positives')
    try:
        viewer = Viewer(args.header, args.z_max, args.tilt_max, args.fps, not args.no_blit)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'Erreur de paramètres : {exc}\n')
    if args.snapshot:
        viewer.save_snapshot(args.snapshot)
        plt.close(viewer.fig)
    else:
        plt.show()


if __name__ == '__main__':
    main()
