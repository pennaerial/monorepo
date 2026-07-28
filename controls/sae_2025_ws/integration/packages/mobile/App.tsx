import { SafeAreaProvider } from "react-native-safe-area-context";
import { WifiScreen } from "./WifiScreen.js";

export default function App() {
  return (
    <SafeAreaProvider>
      <WifiScreen />
    </SafeAreaProvider>
  );
}
