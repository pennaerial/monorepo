const LocationsDisplay = ({ statesToNames }) => {
  return (
    <div className="bg-white rounded-lg shadow-lg border border-gray-200 p-6">
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        {Object.entries(statesToNames).map(([state, names]) => (
          <div
            key={state}
            className="bg-gray-50 rounded-lg p-4 border border-gray-200 hover:shadow-md transition-shadow duration-200"
          >
            {/* Location Header */}
            <h3 className="text-lg font-bold text-gray-800 mb-3">{state}</h3>

            {/* Names List */}
            <div className="space-y-2">
              {names.map((name, index) => (
                <div
                  key={index}
                  className="bg-white px-3 py-2 rounded border border-gray-100"
                >
                  <span className="text-gray-800">{name}</span>
                </div>
              ))}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default LocationsDisplay;
