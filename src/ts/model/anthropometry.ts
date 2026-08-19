export const HAND_DEFAULTS = {
  /** Broad UI limits are intentional and are not inferred from stature. */
  sizeRange: [0.72, 1.32] as const,
  palmLength: 105,
  palmWidth: 54,
  proximalLength: 45,
  middleLength: 28,
  distalLength: 20,
  proximalWidth: 18,
  middleWidth: 15,
  distalWidth: 12,
  mcpRom: [-15, 85] as const,
  pipRom: [0, 105] as const,
  dipRom: [0, 75] as const,
  /** Mean hand-tissue density used by the cylindrical digit approximation. */
  tissueDensityKgPerM3: 1160,
  gravityMPerS2: 9.80665,
};
