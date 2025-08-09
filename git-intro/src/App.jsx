import { useState, useEffect } from "react";
import LocationsDisplay from "./components/LocationsDisplay";
import { statesToNames } from "./data/locations";

function App() {
  const [teamMembers, setTeamMembers] = useState([]);

  useEffect(() => {
    const loadTeamMembers = async () => {
      // Dynamic imports for all profile files
      const modules = import.meta.glob("./team-members/*.js");
      const profiles = [];

      for (const path in modules) {
        const mod = await modules[path]();
        if (mod.profile) {
          profiles.push(mod.profile);
        }
      }

      setTeamMembers(profiles);
    };

    loadTeamMembers();
  }, []);

  return (
    <div className="min-h-screen bg-gray-50">
      <h1 className="text-4xl font-bold text-center py-16">
        penn air software
      </h1>
      <div className="max-w-7xl mx-auto px-4 py-4 space-y-4">
        {/* Team Members Grid */}
        {teamMembers.length > 0 && (
          <div className="bg-white rounded-lg shadow-lg border border-gray-200 p-6">
            <h2 className="text-2xl font-bold text-gray-800 mb-6 text-center">
              👥 Team Member Profiles
            </h2>

            {teamMembers.length === 0 ? (
              <div className="text-center text-gray-600 text-xl font-medium">
                <div className="flex items-center justify-center space-x-3">
                  <div className="animate-spin rounded-full h-6 w-6 border-b-2 border-gray-400"></div>
                  <span>Loading team members...</span>
                </div>
              </div>
            ) : (
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                {teamMembers.map((member, index) => (
                  <div
                    key={index}
                    className="bg-gray-50 rounded-lg shadow border border-gray-200 hover:shadow-md transition-all duration-300 p-6"
                  >
                    {/* Member Header */}
                    <div className="text-center mb-6">
                      <h3 className="text-xl font-bold text-gray-800 mb-3">
                        {member.name}
                      </h3>
                      <div className="bg-gradient-to-r from-blue-600 to-red-600 text-white px-4 py-2 rounded-full text-sm font-bold shadow-md inline-block">
                        {member.grade}
                      </div>
                    </div>

                    {/* Hobbies */}
                    <div className="mb-6">
                      <h4 className="text-lg font-semibold text-gray-800 mb-3">
                        🎯 Hobbies
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

                    {/* Fun Facts */}
                    <div>
                      <h4 className="text-lg font-semibold text-gray-800 mb-3">
                        ✨ Fun Facts
                      </h4>
                      <div className="space-y-3">
                        {member.funFacts.map((fact, factIndex) => (
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
                  </div>
                ))}
              </div>
            )}
          </div>
        )}
      </div>
      {/* Locations Display */}
      <div className="max-w-7xl mx-auto px-4 py-4 space-y-4">
        <LocationsDisplay statesToNames={statesToNames} />
      </div>
    </div>
  );
}

export default App;
