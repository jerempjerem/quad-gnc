import numpy as np
import pytest
from sim import rotations as rot

RNG = np.random.default_rng(0)      # fixed seed : un échec doit être reproductible
IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])

def random_unit_quaternion():
    q = RNG.normal(size=4)
    return q / np.linalg.norm(q)

def random_unit_vector():
    q = RNG.normal(size=3)
    return q / np.linalg.norm(q)

def test_product_identity():

    q = np.array([1.0, 2.0, 3.0, 4.0])

    assert np.allclose(rot.quaternion_product(q, IDENTITY), q)
    assert np.allclose(rot.quaternion_product(IDENTITY, q), q)

def test_product_preserves_norm():
    q1, q2 = random_unit_quaternion(), random_unit_quaternion()

    result = rot.quaternion_product(q1, q2)
    norm = np.linalg.norm(result)
    assert np.isclose(norm, 1.0)
    
def test_product_not_commutative():
    q1, q2 = random_unit_quaternion(), random_unit_quaternion()

    product_1 = rot.quaternion_product(q1, q2)
    product_2 = rot.quaternion_product(q2, q1)

    assert not np.allclose(product_1, product_2)

def test_two_yaw45_equal_yaw90():

    q45 = np.array([np.cos(np.pi/8), 0.0, 0.0, 1.0*np.sin(np.pi/8)])
    expected_q = np.array([np.sqrt(2)/2, 0.0, 0.0, 1.0*np.sqrt(2)/2])

    q90 = rot.quaternion_product(q45, q45)

    assert np.allclose(q90, expected_q)

def test_conjugate():
    q = np.array([1.0, 2.0, 3.0, 4.0])
    expected_q = np.array([1.0, -2.0, -3.0, -4.0])

    assert np.allclose(rot.quaternion_conjugate(q), expected_q)

def test_product_hamilton_ij_equals_k():
    i = np.array([0.0, 1.0, 0.0, 0.0])
    j = np.array([0.0, 0.0, 1.0, 0.0])

    expected = np.array([0.0, 0.0, 0.0, 1.0])

    assert np.allclose(rot.quaternion_product(i, j), expected)

def test_q_product_q_conjugate():
    q = random_unit_quaternion()
    q_conjugate = rot.quaternion_conjugate(q)

    assert np.allclose(rot.quaternion_product(q, q_conjugate), IDENTITY)

def test_rotate_90deg():
    q_nb = np.array([np.sqrt(2)/2, 0.0, 0.0, 1.0*np.sqrt(2)/2])
    v_b = np.array([1.0, 0.0, 0.0])
    expected = np.array([0.0, 1.0, 0.0])

    assert np.allclose(rot.rotate(q_nb, v_b), expected)

def test_rotate_preserves_norm():
    q = random_unit_quaternion()
    v = random_unit_vector()

    assert np.isclose(np.linalg.norm(rot.rotate(q, v)), np.linalg.norm(v))

def test_rotate_conjugate_undoes():
    q_nb = random_unit_quaternion()
    q_bn = rot.quaternion_conjugate(q_nb)
    v = random_unit_vector()

    assert np.allclose(rot.rotate(q_bn, rot.rotate(q_nb, v)), v)

def test_skew():
    a = random_unit_vector()
    b = random_unit_vector()

    assert np.allclose(rot.skew(a) @ b, np.cross(a, b))

def test_quaternion_to_matrix_yaw_90():
    q_90deg_yaw = np.array([np.sqrt(2)/2, 0.0, 0.0, 1.0*np.sqrt(2)/2])

    expected_R = np.array([[0,-1,0],[1,0,0],[0,0,1]])

    assert np.allclose(rot.quaternion_to_matrix(q_90deg_yaw), expected_R)

def test_quaternion_to_matrix_is_rotation():
    R = rot.quaternion_to_matrix(random_unit_quaternion())

    assert np.allclose(R @ R.T, np.eye(3))
    assert np.isclose(np.linalg.det(R), 1.0)

def test_quaternion_to_matrix_matches_rotate():
    q = random_unit_quaternion()
    v = random_unit_vector()

    assert np.allclose(rot.quaternion_to_matrix(q) @ v, rot.rotate(q, v))

def test_matrix_to_quaternion_yaw_90():
    R = np.array([[0,-1,0],[1,0,0],[0,0,1]])

    expected_q = np.array([np.sqrt(2)/2, 0.0, 0.0, 1.0*np.sqrt(2)/2])

    assert np.allclose(rot.matrix_to_quaternion(R), expected_q)

def assert_same_rotation(q1, q2):
    """q et -q codent la même rotation : accepte les deux."""
    assert np.allclose(q1, q2) or np.allclose(q1, -q2)

def test_matrix_to_quaternion_round_trip():
    q = random_unit_quaternion()
    R = rot.quaternion_to_matrix(q)

    assert_same_rotation(rot.matrix_to_quaternion(R), q)

def test_matrix_to_quaternion_rejects_180deg():
    q_180 = np.array([0.0, 1.0, 0.0, 0.0])        # 180° autour de F

    R = rot.quaternion_to_matrix(q_180)

    with pytest.raises(ValueError):
        rot.matrix_to_quaternion(R)

def test_euler_to_quaternion_yaw_90():
    q = rot.euler_to_quaternion(0.0, 0.0, np.pi/2)
    expected_q = np.array([np.sqrt(2)/2, 0.0, 0.0, np.sqrt(2)/2])

    assert_same_rotation(q, expected_q)

def test_euler_to_quaternion_matches_scipy():
    from scipy.spatial.transform import Rotation

    phi, theta, psi = RNG.uniform(-np.pi, np.pi, size=3)

    R_mine = rot.quaternion_to_matrix(rot.euler_to_quaternion(phi, theta, psi))
    R_scipy = Rotation.from_euler("ZYX", [psi, theta, phi]).as_matrix()

    assert np.allclose(R_mine, R_scipy)

def test_quaternion_to_euler_yaw_90():
    q_nb_90 = np.array([np.sqrt(2)/2, 0.0, 0.0, np.sqrt(2)/2])
    assert np.allclose(rot.quaternion_to_euler(q_nb_90), [0.0, 0.0, np.pi/2])

def test_euler_round_trip():
    phi, psi = RNG.uniform(-np.pi, np.pi, size=2)
    theta = RNG.uniform(-np.pi/2 + 0.1, np.pi/2 - 0.1)
    
    q = rot.euler_to_quaternion(phi, theta, psi)

    assert np.allclose(rot.quaternion_to_euler(q), [phi, theta, psi])

def test_integrate_body_axis_after_yaw():
    q0 = rot.euler_to_quaternion(0.0, 0.0, np.pi/2)
    q = rot.integrate_quaternion(q0, np.array([1.0, 0.0, 0.0]), np.pi/2)   # 1 rad/s sur x corps, T = pi/2
    
    assert_same_rotation(q, rot.euler_to_quaternion(np.pi/2, 0.0, np.pi/2))

def test_integrate_n_steps_equals_one_step():
    q0, omega = random_unit_quaternion(), RNG.normal(size=3)
    N, dt = 100, 0.01
    q_n = q0
    for _ in range(N):
        q_n = rot.integrate_quaternion(q_n, omega, dt)
    assert_same_rotation(q_n, rot.integrate_quaternion(q0, omega, N * dt))

def test_integrate_keeps_unit_norm():
    q, omega = random_unit_quaternion(), RNG.normal(size=3)
    for _ in range(10_000):
        q = rot.integrate_quaternion(q, omega, 1e-3)
    assert np.isclose(np.linalg.norm(q), 1.0)

def test_integrate_zero_rate_is_identity():
    q = random_unit_quaternion()
    assert np.allclose(rot.integrate_quaternion(q, np.zeros(3), 0.01), q)