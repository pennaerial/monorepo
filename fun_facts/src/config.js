// Feature flags and app config
// Set VITE_SHOW_GLOBE=true to enable the 3D globe section.
const flag = import.meta.env?.VITE_SHOW_GLOBE;
export const SHOW_GLOBE = flag === 'true' || flag === '1';

