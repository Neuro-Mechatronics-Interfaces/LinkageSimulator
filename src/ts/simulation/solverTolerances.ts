/** Shared numerical policy for the planar mechanism constraint solver. */
export const SOLVER_TOLERANCES = {
  length: 1e-6,
  closure: 1e-5,
  lockedAngle: 1e-7,
  jointLimit: 1e-7,
  finiteDifferenceTranslation: 1e-5,
  finiteDifferenceAngle: 1e-6,
  rankAbsolute: 1e-8,
  rankRelative: 1e-7,
  numericalResidual: 1e-5,
} as const;

/**
 * Converts angular residuals to length-like residuals before least-squares
 * operations. Fifty millimetres is representative of the demonstrator links,
 * so one radian has a closure penalty comparable to a 50 mm point error.
 */
export const ANGLE_RESIDUAL_LENGTH_SCALE = 50;

