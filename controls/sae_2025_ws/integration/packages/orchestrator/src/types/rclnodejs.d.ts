// rclnodejs is an optional dependency (see package.json) that only builds in
// a sourced ROS2 environment, so its own types aren't reliably available here.
// This ambient declaration lets `import("rclnodejs")` typecheck regardless of
// whether the real package is actually installed; trigger-service-client.ts
// re-types its actual shape locally instead of trusting this blindly.
declare module "rclnodejs";
