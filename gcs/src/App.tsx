import { useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import FoxgloveClient from "./foxglove-ws/foxglove-client";
import { IWebSocket } from "./foxglove-ws/types";
import "./App.css";



function App() {
  const [greetMsg, setGreetMsg] = useState("");
  const [name, setName] = useState("");


  async function connect() {
    const ws = new WebSocket("ws://localhost:8765", [FoxgloveClient.SUPPORTED_SUBPROTOCOL]);
    const client = new FoxgloveClient({ ws: ws as IWebSocket });

    client.on("open", () => {
      console.log("foxglove connected");
    });
    client.on("advertise", (newChannels) => {
      for (const channel of newChannels) {
        console.log(JSON.stringify(channel, null, 2));
      }
    });

    // client.subscribe()
    // Learn more about Tauri commands at https://tauri.app/develop/calling-rust/
    // setGreetMsg(await invoke("greet", { name }));
  }

  return (
    <main className="container">
      <h1>Welcome to PennAiR GCS</h1>

      <form
        className="row"
        onSubmit={(e) => {
          e.preventDefault();
          connect();
        }}
      >
        <input
          id="greet-input"
          onChange={(e) => {
            setName(e.currentTarget.value);
            console.log("PRESSED");
          }}
          placeholder="Enter a name..."
        />
        <button type="submit">Connect</button>
      </form>
      <p>{greetMsg}</p>
    </main>
  );
}

export default App;
