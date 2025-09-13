// Minimal geodata helpers for mapping locations to approximate coordinates.
// Supports:
// - US states via centroid lat/lng
// - Common countries via centroid lat/lng
// - Allows profiles to override with explicit location.lat/lng

export const US_STATE_CENTROIDS = {
  Alabama: { lat: 32.806671, lng: -86.79113 },
  Alaska: { lat: 64.20084, lng: -149.49367 },
  Arizona: { lat: 34.048927, lng: -111.093735 },
  Arkansas: { lat: 34.969704, lng: -92.373123 },
  California: { lat: 36.778259, lng: -119.417931 },
  Colorado: { lat: 39.550053, lng: -105.782066 },
  Connecticut: { lat: 41.603221, lng: -73.087749 },
  Delaware: { lat: 38.910832, lng: -75.52767 },
  "District of Columbia": { lat: 38.905985, lng: -77.033418 },
  Florida: { lat: 27.664827, lng: -81.515755 },
  Georgia: { lat: 32.165622, lng: -82.900078 },
  Hawaii: { lat: 19.896767, lng: -155.582779 },
  Idaho: { lat: 44.068203, lng: -114.742043 },
  Illinois: { lat: 40.633125, lng: -89.398529 },
  Indiana: { lat: 40.267194, lng: -86.134902 },
  Iowa: { lat: 41.878003, lng: -93.097702 },
  Kansas: { lat: 39.011902, lng: -98.484245 },
  Kentucky: { lat: 37.839333, lng: -84.27002 },
  Louisiana: { lat: 30.984299, lng: -91.962334 },
  Maine: { lat: 45.253783, lng: -69.445469 },
  Maryland: { lat: 39.045753, lng: -76.641273 },
  Massachusetts: { lat: 42.407211, lng: -71.382439 },
  Michigan: { lat: 44.314842, lng: -85.602364 },
  Minnesota: { lat: 46.729553, lng: -94.685898 },
  Mississippi: { lat: 32.354668, lng: -89.398529 },
  Missouri: { lat: 37.964253, lng: -91.831833 },
  Montana: { lat: 46.879681, lng: -110.362564 },
  Nebraska: { lat: 41.492538, lng: -99.90181 },
  Nevada: { lat: 38.802609, lng: -116.419388 },
  "New Hampshire": { lat: 43.193852, lng: -71.572395 },
  "New Jersey": { lat: 40.058323, lng: -74.405663 },
  "New Mexico": { lat: 34.51994, lng: -105.870087 },
  "New York": { lat: 43.299428, lng: -74.217933 },
  "North Carolina": { lat: 35.759575, lng: -79.019302 },
  "North Dakota": { lat: 47.551487, lng: -101.002014 },
  Ohio: { lat: 40.417286, lng: -82.90712 },
  Oklahoma: { lat: 35.007751, lng: -97.092026 },
  Oregon: { lat: 43.804133, lng: -120.554201 },
  Pennsylvania: { lat: 41.203323, lng: -77.194527 },
  "Rhode Island": { lat: 41.580095, lng: -71.477429 },
  "South Carolina": { lat: 33.836082, lng: -81.163727 },
  "South Dakota": { lat: 43.969517, lng: -99.90181 },
  Tennessee: { lat: 35.517491, lng: -86.580444 },
  Texas: { lat: 31.968599, lng: -99.90181 },
  Utah: { lat: 39.32098, lng: -111.093735 },
  Vermont: { lat: 44.558803, lng: -72.577843 },
  Virginia: { lat: 37.431573, lng: -78.656891 },
  Washington: { lat: 47.751076, lng: -120.740135 },
  "West Virginia": { lat: 38.597626, lng: -80.454903 },
  Wisconsin: { lat: 43.78444, lng: -88.787868 },
  Wyoming: { lat: 43.075968, lng: -107.290283 },
};

export const COUNTRY_CENTROIDS = {
  "United States": { lat: 39.8283, lng: -98.5795 },
  Canada: { lat: 56.1304, lng: -106.3468 },
  Mexico: { lat: 23.6345, lng: -102.5528 },
  Brazil: { lat: -14.235, lng: -51.9253 },
  "United Kingdom": { lat: 55.3781, lng: -3.436 },
  Ireland: { lat: 53.1424, lng: -7.6921 },
  France: { lat: 46.2276, lng: 2.2137 },
  Germany: { lat: 51.1657, lng: 10.4515 },
  Spain: { lat: 40.4637, lng: -3.7492 },
  Italy: { lat: 41.8719, lng: 12.5674 },
  Netherlands: { lat: 52.1326, lng: 5.2913 },
  Belgium: { lat: 50.5039, lng: 4.4699 },
  Switzerland: { lat: 46.8182, lng: 8.2275 },
  Sweden: { lat: 60.1282, lng: 18.6435 },
  Norway: { lat: 60.472, lng: 8.4689 },
  Denmark: { lat: 56.2639, lng: 9.5018 },
  Poland: { lat: 51.9194, lng: 19.1451 },
  Portugal: { lat: 39.3999, lng: -8.2245 },
  Russia: { lat: 61.524, lng: 105.3188 },
  Ukraine: { lat: 48.3794, lng: 31.1656 },
  Turkey: { lat: 38.9637, lng: 35.2433 },
  Israel: { lat: 31.0461, lng: 34.8516 },
  "Saudi Arabia": { lat: 23.8859, lng: 45.0792 },
  UAE: { lat: 23.4241, lng: 53.8478 },
  India: { lat: 20.5937, lng: 78.9629 },
  China: { lat: 35.8617, lng: 104.1954 },
  Japan: { lat: 36.2048, lng: 138.2529 },
  "South Korea": { lat: 35.9078, lng: 127.7669 },
  Singapore: { lat: 1.3521, lng: 103.8198 },
  Australia: { lat: -25.2744, lng: 133.7751 },
  "New Zealand": { lat: -40.9006, lng: 174.886 },
  "South Africa": { lat: -30.5595, lng: 22.9375 },
  Nigeria: { lat: 9.082, lng: 8.6753 },
  Kenya: { lat: -0.0236, lng: 37.9062 },
  Egypt: { lat: 26.8206, lng: 30.8025 },
  Argentina: { lat: -38.4161, lng: -63.6167 },
  Chile: { lat: -35.6751, lng: -71.543 },
  Colombia: { lat: 4.5709, lng: -74.2973 },
  Peru: { lat: -9.19, lng: -75.0152 },
  Venezuela: { lat: 6.4238, lng: -66.5897 },
};

export function resolveLatLng(location = {}) {
  const { lat, lng, country, state } = location;
  if (typeof lat === "number" && typeof lng === "number") {
    return { lat, lng };
  }
  // Normalize common US aliases
  let c = (country || "").trim();
  const usAliases = new Set(["US", "USA", "United States of America", "United States"]);
  if (usAliases.has(c)) c = "United States";
  if (c === "United States" && state && US_STATE_CENTROIDS[state]) {
    return US_STATE_CENTROIDS[state];
  }
  if (c && COUNTRY_CENTROIDS[c]) return COUNTRY_CENTROIDS[c];
  return null; // couldn't resolve
}
