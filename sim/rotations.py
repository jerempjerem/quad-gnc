import numpy as np

def quaternion_product(q1: np.ndarray[np.float64], q2: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    if q1.shape != (4, ) or q2.shape != (4, ):
        raise ValueError("Invalid quaternion shape")

    result = np.zeros((4, ))
    q1_w = q1[0]
    q2_w = q2[0]
    q1_v = q1[1:]
    q2_v = q2[1:]
    
    result[0] = q1_w*q2_w - q1_v @ q2_v
    result[1:] = q1_w*q2_v + q2_w*q1_v + np.cross(q1_v, q2_v)

    return result

def quaternion_conjugate(q: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    if q.shape != (4, ):
            raise ValueError("Invalid quaternion shape")

    q_copy = q.copy()
    
    q_copy[1:] = -q_copy[1:]

    return q_copy

def rotate(q: np.ndarray[np.float64], v: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    if q.shape != (4, ):
        raise ValueError("Invalid quaternion shape")

    if v.shape != (3, ):
        raise ValueError("Invalid vector shape")

    v_extended = np.zeros((4,))
    v_extended[1:] = v

    q_conjugate = quaternion_conjugate(q)

    product_1 = quaternion_product(q, v_extended)
    result = quaternion_product(product_1, q_conjugate)

    return result[1:]

def skew(q: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    if q.shape != (3, ):
        raise ValueError("Invalid quaternion shape")

    q_x = q[0]
    q_y = q[1]
    q_z = q[2]

    return np.array([
        [0, -q_z, q_y],
        [q_z, 0, -q_x],
        [-q_y, q_x, 0]
    ])

    
def quaternion_to_matrix(q: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    if q.shape != (4, ):
        raise ValueError("Invalid quaternion shape")

    q_w = q[0]
    q_v = q[1:]

    R = (q_w**2 - q_v @ q_v) * np.identity(3) + 2 * np.outer(q_v, q_v) + 2 * q_w * skew(q_v)

    return R

def matrix_to_quaternion(R: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    """Quaternion q (w, x, y, z) tel que quaternion_to_matrix(q) == R, avec q_w >= 0.

    Méthode par la trace : q_w = sqrt((tr R + 1) / 4), q_v lu sur R - R^T.
    Domaine : lève ValueError si |θ - 180°| < 0.11° (q_w < 1e-3), où la
    division par q_w est mal conditionnée.
    """
    if R.shape != (3, 3):
        raise ValueError("Invalid matrix shape")

    q = np.zeros((4, ))


    t = R.trace() + 1
    # rejette |θ − 180°| < 0.11° (q_w < 1e-3),
    if t < 4e-6:
        raise ValueError("Invalid q_w error")
    
    q_w = np.sqrt(t/4) #q_w
    
    diff = R - R.transpose()

    divider = 4 * q_w
    q_x = diff[2][1] / divider
    q_y = diff[0][2] / divider
    q_z = diff[1][0] / divider

    q[0] = q_w
    q[1] = q_x
    q[2] = q_y
    q[3] = q_z

    return q

def build_quaternion(angle: np.float64, u: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    if u.shape != (3,):
        raise ValueError("Invalid u vector shape")

    norm_u = np.linalg.norm(u)
    if norm_u == 0:
        raise ValueError("Invalid u norm")

    normalized_u = u / norm_u
    
    q = np.zeros((4, ))

    q[0] = np.cos(angle/2)
    q[1:] = normalized_u * np.sin(angle/2)

    return q

def euler_to_quaternion(phi: np.float64, theta: np.float64, psi: np.float64) -> np.ndarray[np.float64]:
    """ZYX convention"""

    # q_z(ψ)
    q_z = build_quaternion(psi, np.array([0.0, 0.0, 1.0]))

    # q_y(θ)
    q_y = build_quaternion(theta, np.array([0.0, 1.0, 0.0]))

    # ⊗q_x(φ)
    q_x = build_quaternion(phi, np.array([1.0, 0.0, 0.0]))

    # q_z(ψ)⊗q_y(θ)⊗q_x(φ)
    q = quaternion_product(quaternion_product(q_z, q_y), q_x)

    return q

def quaternion_to_euler(q: np.ndarray[np.float64]) -> np.ndarray[np.float64]:
    """Angles d'Euler (phi, theta, psi) en radians du quaternion q.

    Séquence 3-2-1 intrinsèque : R_nb = R_z(psi).R_y(theta).R_x(phi).
    Sortie : np.array([phi, theta, psi]), theta dans [-pi/2, pi/2],
    phi et psi dans (-pi, pi].
    Indéfini à |theta| = pi/2 (blocage de cardan)
    """

    R = quaternion_to_matrix(q)

    
    theta = -np.arcsin(np.clip(R[2][0], -1, 1)) # a cause des arrondi R[2][0] 
    phi = np.arctan2(R[2][1], R[2][2])
    psi = np.arctan2(R[1][0], R[0][0])

    return np.array([phi, theta, psi])


def integrate_quaternion(
        q: np.ndarray[np.float64], 
        omega: np.ndarray[np.float64], 
        dt: np.float64
    ) -> np.ndarray[np.float64]:
    """intégration d'un pas sous hypothèse de ω constante sur [t, t + Δt]"""

    if q.shape != (4, ):
        raise ValueError("invalid quaternion shape")

    if omega.shape != (3, ):
        raise ValueError("invalid omega shape")

    omega_norm = np.linalg.norm(omega)
    if omega_norm * dt == 0:
        return q.copy()

    q_wt = build_quaternion(omega_norm * dt, omega)

    q_n1 = quaternion_product(q, q_wt)

    return q_n1 / np.linalg.norm(q_n1)

    
