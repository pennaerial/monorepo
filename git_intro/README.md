# Git Intro Demo (Onboarding)

This small React app helps onboard new members: add your profile, show up on the page, and make your first PR.

## Run Locally

- From this folder, run: `npm install` (first time) and `npm run dev`
- Open the local URL Vite prints out

## Add Yourself (Your First PR)

1) Create your profile file

- Copy `src/team-members/PROFILE_TEMPLATE.js` to a new file, e.g. `src/team-members/janeProfile.js`
- Fill out the fields. We use a Class object (not a free-text grade). Locations support both US and international students:

```
export const profile = {
  name: "Jane Doe",
  class: { year: 2028, program: "SEAS" },
  // If you're in the US:
  location: { country: "United States", state: "Pennsylvania" },
  hobbies: ["Climbing", "Cooking"],
  funFacts: ["I’ve flown a glider!"],
};
```

International example:

```
export const profile = {
  name: "Alex Li",
  class: { year: 2027, program: "SEAS" },
  location: { country: "China", city: "Shanghai" }, // city/region optional
  hobbies: ["Badminton"],
  funFacts: ["I can fold a crane in 20s"],
};
```

Optional: add exact coordinates for precise pin placement on the globe:

```
location: { country: "United States", state: "Pennsylvania", lat: 39.9526, lng: -75.1652 }
```

2) Home state for the map

- No separate file to touch — the map is generated from your `profile.location`.

3D Globe

- The “Where We’re From” section renders a custom Three.js 3D globe with pins.
- Feature flag: set `VITE_SHOW_GLOBE=true` to display the globe; otherwise the section is hidden and each member card adds a "Location: …" line under Fun Facts.
- The globe uses a procedural ocean base and supports an optional transparent land-mask overlay (black land, transparent ocean) for a clean, stylized look.
- Hover a pin to see who’s there. If your country or state isn’t recognized, add `lat`/`lng` to your profile for precise placement.
- You can center a different meridian by passing a `longitudeOffset` to the globe (e.g., 180 for Pacific-centered), and `centerLat/centerLon` to face a specific location.

3) Check the app

- Run `npm run dev` to verify your card appears. You can search and sort by class.

4) Open a PR

- Create a branch, commit your changes, push, and open a PR to `main`.

## Notes

- The app auto-discovers all files in `src/team-members/*.js` that export `profile` (it ignores `PROFILE_TEMPLATE.js`).
- We migrated from a free-text `grade` field to a structured `class` object: `{ year: number, program?: string }`.
- `location.state` powers the “Where We’re From” section — no more manual `src/data/locations.js` edits.
- Backwards compatibility: old profiles with `grade` will still render, but please use `class` for all new profiles.
