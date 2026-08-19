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

export const DIGIT_IDS = ['d2', 'd3', 'd4', 'd5'] as const;

/**
 * Rounded neutral-adult proportions relative to D2. They are visualization
 * defaults rather than subject-specific estimates; the shared hand-size
 * multiplier is applied independently of these between-digit proportions.
 */
export const DIGIT_PROFILES = {
  d2: { name: 'D2 · Index', lengthScale: [1, 1, 1] as const, widthScale: 1 },
  d3: { name: 'D3 · Middle', lengthScale: [1.11, 1.11, 1.11] as const, widthScale: 1.11 },
  d4: { name: 'D4 · Ring', lengthScale: [1.03, 1.03, 1.03] as const, widthScale: 1.03 },
  d5: { name: 'D5 · Little', lengthScale: [0.83, 0.83, 0.83] as const, widthScale: 0.83 },
} as const;
