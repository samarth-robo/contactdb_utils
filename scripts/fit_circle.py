"""
Functions for fitting a circle to 3D points.
Source https://meshlogic.github.io/posts/jupyter/curve-fitting/fitting-a-circle-to-cluster-of-3d-points/
"""

from numpy import *

# -------------------------------------------------------------------------------
# FIT CIRCLE 2D
# - Find center [xc, yc] and radius r of circle fitting to set of 2D points
# - Optionally specify weights for points
#
# - Implicit circle function:
#   (x-xc)^2 + (y-yc)^2 = r^2
#   (2*xc)*x + (2*yc)*y + (r^2-xc^2-yc^2) = x^2+y^2
#   c[0]*x + c[1]*y + c[2] = x^2+y^2
#
# - Solution by method of least squares:
#   A*c = b, c' = argmin(||A*c - b||^2)
#   A = [x y 1], b = [x^2+y^2]
# -------------------------------------------------------------------------------
def fit_circle_2d(x, y, w=[]):
  A = array([x, y, ones(len(x))]).T
  b = x ** 2 + y ** 2

  # Modify A,b for weighted least squares
  if len(w) == len(x):
    W = diag(w)
    A = dot(W, A)
    b = dot(W, b)

  # Solve by method of least squares
  c = linalg.lstsq(A, b, rcond=None)[0]

  # Get circle parameters from solution c
  xc = c[0] / 2
  yc = c[1] / 2
  r = sqrt(c[2] + xc ** 2 + yc ** 2)
  return xc, yc, r


# -------------------------------------------------------------------------------
# RODRIGUES ROTATION
# - Rotate given points based on a starting and ending vector
# - Axis k and angle of rotation theta given by vectors n0,n1
#   P_rot = P*cos(theta) + (k x P)*sin(theta) + k*<k,P>*(1-cos(theta))
# -------------------------------------------------------------------------------
def rodrigues_rot(P, n0, n1):
  # If P is only 1d array (coords of single point), fix it to be matrix
  if P.ndim == 1:
    P = P[newaxis, :]

  # Get vector of rotation k and angle theta
  n0 = n0 / linalg.norm(n0)
  n1 = n1 / linalg.norm(n1)
  k = cross(n0, n1)
  k = k / linalg.norm(k)
  theta = arccos(dot(n0, n1))

  # Compute rotated points
  P_rot = zeros((len(P), 3))
  for i in range(len(P)):
    P_rot[i] = P[i] * cos(theta) + cross(k, P[i]) * sin(theta) + k * dot(k,
      P[i]) * (1 - cos(theta))

  return P_rot


# -------------------------------------------------------------------------------
# ANGLE BETWEEN
# - Get angle between vectors u,v with sign based on plane with unit normal n
# -------------------------------------------------------------------------------
def angle_between(u, v, n=None):
  if n is None:
    return arctan2(linalg.norm(cross(u, v)), dot(u, v))
  else:
    return arctan2(dot(n, cross(u, v)), dot(u, v))


# -------------------------------------------------------------------------------
# - Make axes of 3D plot to have equal scales
# - This is a workaround to Matplotlib's set_aspect('equal') and axis('equal')
#   which were not working for 3D
# -------------------------------------------------------------------------------
def set_axes_equal_3d(ax):
  limits = array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
  spans = abs(limits[:, 0] - limits[:, 1])
  centers = mean(limits, axis=1)
  radius = 0.5 * max(spans)
  ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
  ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
  ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def fit_circle_3d(P):
  """
  :param P: 3D points
  :return: center, radius, normal vector
  """
  # -------------------------------------------------------------------------------
  # (1) Fitting plane by SVD for the mean-centered data
  # Eq. of plane is <p,n> + d = 0, where p is a point on plane and n is normal vector
  # -------------------------------------------------------------------------------
  P_mean = P.mean(axis=0)
  P_centered = P - P_mean
  U, s, V = linalg.svd(P_centered)

  # Normal vector of fitting plane is given by 3rd column in V
  # Note linalg.svd returns V^T, so we need to select 3rd row from V^T
  normal = V[2, :]
  d = -dot(P_mean, normal)  # d = -<p,n>

  # -------------------------------------------------------------------------------
  # (2) Project points to coords X-Y in 2D plane
  # -------------------------------------------------------------------------------
  P_xy = rodrigues_rot(P_centered, normal, [0, 0, 1])

  # -------------------------------------------------------------------------------
  # (3) Fit circle in new 2D coords
  # -------------------------------------------------------------------------------
  xc, yc, r = fit_circle_2d(P_xy[:, 0], P_xy[:, 1])

  # -------------------------------------------------------------------------------
  # (4) Transform circle center back to 3D coords
  # -------------------------------------------------------------------------------
  C = rodrigues_rot(array([xc, yc, 0]), [0, 0, 1], normal) + P_mean
  C = C.flatten()

  return C, r, normal

def get_align_rotmat(n1, n2 = array([0,0,1])):
  """
  gives rotation matrix that aligns the n1 vector to n2 vector
  :param n1:
  :return:
  """
  assert n1.size == 3
  n1 /= linalg.norm(n1)

  v = cross(n2, n1)
  vx = zeros((4,4))
  vx[0, 1] = -v[2]
  vx[1, 0] = +v[2]
  vx[0, 2] = +v[1]
  vx[2, 0] = -v[1]
  vx[1, 2] = -v[0]
  vx[2, 1] = +v[0]

  dotp = dot(n1, n2)

  T = eye(4) + vx + dot(vx, vx)/(1 + dotp)

  return T
