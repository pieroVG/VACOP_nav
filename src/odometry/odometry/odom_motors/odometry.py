# odometry.py
import time
import math

class DifferentialOdometry:
    def __init__(
        self,
        motors,
        wheel_radius: float,
        wheel_base: float,
        gear_ratio: float = 1.0,
        left_sign: int = 1,
        right_sign: int = 1,
    ):
        """
        motors       : instance de DualMotorController
        wheel_radius : rayon de roue (m)
        wheel_base   : distance entre roues (m)
        gear_ratio   : réduction mécanique (>=1.0)
        left_sign    : +1 ou -1 selon montage roue gauche
        right_sign   : +1 ou -1 selon montage roue droite
        """
        self.motors = motors

        self.R = wheel_radius
        self.L = wheel_base
        self.gear = gear_ratio

        self.left_sign = 1 if left_sign >= 0 else -1
        self.right_sign = 1 if right_sign >= 0 else -1

        # État odométrique
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # rad
        self.v = 0.0       # m/s (vitesse linéaire)
        self.omega = 0.0   # rad/s (vitesse angulaire)
        self.a_linear = 0.0   # m/s²
        self.a_angular = 0.0  # rad/s²

        # pour calculer dérivée de la vitesse
        self._last_v = 0.0
        self._last_omega = 0.0
        self._last_time = time.time()

        print(
            f"[ODOM] Init: R={self.R}m L={self.L}m "
            f"gear={self.gear} signs(L={self.left_sign},R={self.right_sign})"
        )

    # --------------------------------------------------
    # API publique
    # --------------------------------------------------

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """Réinitialise la pose estimée"""
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.a_linear = 0.0
        self.a_angular = 0.0
        self._last_v = 0.0
        self._last_omega = 0.0
        self._last_time = time.time()
        print(
            f"[ODOM] Reset: x={x:.2f}, y={y:.2f}, "
            f"theta={math.degrees(theta):.1f}°"
        )

    def update(self):
        """À appeler périodiquement (ex: 50 Hz)"""
        now = time.time()
        dt = now - self._last_time
        if dt <= 0.0 or dt > 0.5:
            self._last_time = now
            return

        self._last_time = now

        # Lire vitesses roues (RPM)
        try:
            left_rpm, right_rpm = self.motors.get_wheel_speeds_rpm()
        except Exception:
            return

        left_rpm *= self.left_sign
        right_rpm *= self.right_sign

        # RPM -> rad/s
        wl = left_rpm * 2.0 * math.pi / 60.0
        wr = right_rpm * 2.0 * math.pi / 60.0

        # rad/s -> m/s
        vl = (wl * self.R) / self.gear
        vr = (wr * self.R) / self.gear

        # Cinématique différentielle
        v_new = (vr + vl) / 2.0
        omega_new = (vr - vl) / self.L

        # Calcul des accélérations
        self.a_linear = (v_new - self._last_v) / dt
        self.a_angular = (omega_new - self._last_omega) / dt

        self._last_v = v_new
        self._last_omega = omega_new

        self.v = v_new
        self.omega = omega_new

        # Intégration pour position
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += self.v * dt * math.cos(self.theta)
        self.y += self.v * dt * math.sin(self.theta)

    def get_pose(self):
        """Retourne la pose complète sous forme de dictionnaire"""
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,          # orientation
            "v_linear": self.v,           # vitesse linéaire
            "v_angular": self.omega,      # vitesse angulaire
            "a_linear": self.a_linear,    # accélération linéaire
            "a_angular": self.a_angular,  # accélération angulaire
            "timestamp": time.time(),
        }

    def get_pose_tuple(self):
        """Retourne la pose sous forme de tuple (x, y, theta, v, omega, a_lin, a_ang)"""
        return self.x, self.y, self.theta, self.v, self.omega, self.a_linear, self.a_angular
    
    def get_wheel_speeds_rpm(self):
        """
        m1 = roue gauche
        m2 = roue droite
        """
        # display_speed() 
        left_rpm = 0.0
        right_rpm = 0.0
        try:
            if self.m1: left_rpm = self.m1.get_speed_rpm()
            if self.m2: right_rpm = self.m2.get_speed_rpm()
        except Exception as e:
            self._print("WARN get_wheel_speeds_rpm:", e)
        return left_rpm, right_rpm
