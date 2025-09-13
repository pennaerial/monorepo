import { useState, useEffect, useMemo } from "react";
import Globe from "./components/Globe";
import { SHOW_GLOBE } from "./config";
import landMask from "./assets/land-mask.png";
import { resolveLatLng } from "./data/geo";

// Helpers
const getInitials = (name = "") =>
  name
    .split(" ")
    .filter(Boolean)
    .slice(0, 2)
    .map((n) => n[0]?.toUpperCase())
    .join("");

const formatClass = (member) => {
  if (member?.class?.year) {
    return `Class of ${member.class.year}`;
  }
  if (member?.grade) return member.grade; // Back-compat with older profiles
  return "Class";
};

function App() {
  const [teamMembers, setTeamMembers] = useState([]);
  const [query, setQuery] = useState("");
  const [sortBy, setSortBy] = useState("name");

  useEffect(() => {
    const loadTeamMembers = async () => {
      // Dynamic imports for all profile files
      const modules = import.meta.glob("./team-members/*.js");
      const profiles = [];

      for (const path in modules) {
        // Skip the template so it never renders
        if (path.endsWith("/PROFILE_TEMPLATE.js")) continue;
        const mod = await modules[path]();
        if (mod.profile) {
          profiles.push(mod.profile);
        }
      }

      setTeamMembers(profiles);
    };

    loadTeamMembers();
  }, []);

  const filteredMembers = useMemo(() => {
    const q = query.trim().toLowerCase();
    let list = [...teamMembers];
    if (q) {
      list = list.filter(
        (m) =>
          m.name?.toLowerCase().includes(q) ||
          m.hobbies?.some((h) => h.toLowerCase().includes(q))
      );
    }
    if (sortBy === "name") {
      list.sort((a, b) => a.name.localeCompare(b.name));
    } else if (sortBy === "class") {
      const byYearDesc = (a, b) => (b.class?.year ?? 0) - (a.class?.year ?? 0);
      list.sort((a, b) => {
        const diff = byYearDesc(a, b);
        return diff !== 0 ? diff : a.name.localeCompare(b.name);
      });
    }
    return list;
  }, [teamMembers, query, sortBy]);

  // (UI now uses 3D globe only; list mapping removed)

  return (
    <div className="min-h-screen bg-gradient-to-b from-slate-50 to-white">
      {/* Hero */}
      <header className="relative overflow-hidden">
        <div className="absolute inset-0 bg-[radial-gradient(ellipse_at_top,_var(--tw-gradient-stops))] from-blue-100 via-transparent to-transparent" />
        <div className="relative max-w-7xl mx-auto px-4 pt-14 pb-10 text-center">
          <h1 className="text-4xl md:text-5xl font-extrabold tracking-tight">
            Penn Air Software
          </h1>
          <p className="mt-4 text-gray-600 max-w-2xl mx-auto">
            Meet the team and make your first PR. Add your profile, share a fun fact, and join the flight.
          </p>
        </div>
      </header>

      <div className="max-w-7xl mx-auto px-4 py-6 space-y-6">
        {/* Team Members Grid */}
        <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-6">
          <h2 className="text-2xl font-bold text-gray-800 mb-6 text-center">
            👥 Team Member Profiles
          </h2>

          {/* Controls */}
          <div className="flex flex-col md:flex-row gap-3 md:items-center md:justify-between mb-4">
            <input
              type="text"
              placeholder="Search by name or hobby..."
              value={query}
              onChange={(e) => setQuery(e.target.value)}
              className="w-full md:w-72 rounded-lg border border-gray-300 px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
            <div className="flex items-center gap-2">
              <label htmlFor="sort" className="text-sm text-gray-600">Sort by:</label>
              <select
                id="sort"
                value={sortBy}
                onChange={(e) => setSortBy(e.target.value)}
                className="rounded-lg border border-gray-300 px-3 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-blue-500"
              >
                <option value="name">Name (A–Z)</option>
                <option value="class">Class (newest)</option>
              </select>
            </div>
          </div>

          {teamMembers.length === 0 ? (
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
              {Array.from({ length: 6 }).map((_, i) => (
                <div key={i} className="animate-pulse bg-gray-50 rounded-lg border border-gray-200 p-6">
                  <div className="flex items-center gap-4 mb-6">
                    <div className="h-12 w-12 rounded-full bg-gray-200" />
                    <div className="flex-1">
                      <div className="h-4 bg-gray-200 rounded w-2/3 mb-2" />
                      <div className="h-3 bg-gray-200 rounded w-1/3" />
                    </div>
                  </div>
                  <div className="h-3 bg-gray-200 rounded w-1/2 mb-3" />
                  <div className="flex gap-2">
                    <div className="h-6 bg-gray-200 rounded w-16" />
                    <div className="h-6 bg-gray-200 rounded w-20" />
                    <div className="h-6 bg-gray-200 rounded w-12" />
                  </div>
                </div>
              ))}
            </div>
          ) : (
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
              {filteredMembers.map((member, index) => (
                <div
                  key={index}
                  className="bg-gray-50 rounded-lg shadow-sm border border-gray-200 hover:shadow-md transition-all duration-300 p-6"
                >
                  {/* Member Header */}
                  <div className="flex flex-col items-center text-center mb-6">
                    <div className="h-12 w-12 rounded-full bg-gradient-to-br from-blue-600 to-red-600 text-white font-bold flex items-center justify-center shadow-md">
                      {getInitials(member.name)}
                    </div>
                    <h3 className="mt-3 text-xl font-bold text-gray-800">
                      {member.name}
                    </h3>
                    <div className="mt-2 bg-blue-600/10 text-blue-800 px-3 py-1 rounded-full text-xs font-semibold border border-blue-200">
                      🎓 {formatClass(member)}{member?.class?.program ? ` • ${member.class.program}` : ""}
                    </div>
                  </div>

                  {/* Hobbies */}
                  {Array.isArray(member.hobbies) && member.hobbies.length > 0 && (
                    <div className="mb-6">
                      <h4 className="text-sm font-semibold text-gray-700 mb-2 uppercase tracking-wide">
                        Hobbies
                      </h4>
                      <div className="flex flex-wrap gap-2">
                        {member.hobbies.map((hobby, hobbyIndex) => (
                          <span
                            key={hobbyIndex}
                            className="bg-blue-50 text-blue-800 px-3 py-1 rounded-full text-xs font-medium border border-blue-200 hover:bg-blue-100 transition-colors duration-200"
                          >
                            {hobby}
                          </span>
                        ))}
                      </div>
                    </div>
                  )}

                  {/* Fun Facts (with optional Location when globe is hidden) */}
                  {(() => {
                    const loc = member?.location || {};
                    const parts = [];
                    if (loc.city) parts.push(loc.city);
                    if (loc.region) parts.push(loc.region);
                    if (loc.state) parts.push(loc.state);
                    if (loc.country) parts.push(loc.country);
                    let locationLabel = parts.join(", ");
                    if (!locationLabel && typeof loc.lat === "number" && typeof loc.lng === "number") {
                      locationLabel = `${loc.lat.toFixed(3)}, ${loc.lng.toFixed(3)}`;
                    }
                    const extraFacts = !SHOW_GLOBE && locationLabel ? [
                      `From: ${locationLabel}`,
                    ] : [];
                    const baseFacts = Array.isArray(member.funFacts) ? member.funFacts : [];
                    const allFacts = [...baseFacts, ...extraFacts];
                    if (allFacts.length === 0) return null;
                    return (
                      <div>
                        <h4 className="text-sm font-semibold text-gray-700 mb-2 uppercase tracking-wide">
                          Fun Facts
                        </h4>
                        <div className="space-y-3">
                          {allFacts.map((fact, factIndex) => (
                            <div
                              key={factIndex}
                              className="bg-white p-3 rounded-lg border border-gray-200 shadow-sm hover:shadow-md transition-all duration-200"
                            >
                              <p className="text-gray-700 text-sm leading-relaxed">
                                {fact}
                              </p>
                            </div>
                          ))}
                        </div>
                      </div>
                    );
                  })()}
                </div>
              ))}
            </div>
          )}
        </div>

        {/* Locations Display (conditional) */}
        {SHOW_GLOBE && (
          <div className="space-y-6">
            <h2 className="text-2xl font-bold text-gray-800 text-center">🌍 Where We're From</h2>
            {/* 3D Globe with pins */}
            <div className="bg-white rounded-xl shadow-lg border border-gray-200 p-4">
              <Globe
                height={560}
                landMask={landMask}
                longitudeOffset={0}
                centerLat={11.851223}
                centerLon={13.794623}
                landMaskProjection="equirect"
                flipLongitude={false}
                debugLogOnClick={false}
                pins={(() => {
                  // group members by resolved coordinate to aggregate names per place
                  const groups = new Map();
                  for (const m of teamMembers) {
                    const loc = m?.location || {};
                    const coords = resolveLatLng(loc);
                    if (!coords) continue;
                    const key = `${coords.lat.toFixed(3)},${coords.lng.toFixed(3)}`;
                    const labelParts = [];
                    if (loc.city) labelParts.push(loc.city);
                    if (loc.region) labelParts.push(loc.region);
                    if (loc.state) labelParts.push(loc.state);
                    if (loc.country) labelParts.push(loc.country);
                    const label = labelParts.length ? labelParts.join(", ") : "Location";
                    if (!groups.has(key)) {
                      groups.set(key, { lat: coords.lat, lng: coords.lng, label, members: [] });
                    }
                    groups.get(key).members.push(m.name);
                  }
                  return Array.from(groups.values());
                })()}
              />
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

export default App;
