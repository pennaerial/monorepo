import { useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import FoxgloveClient from "./foxglove-ws/foxglove-client";
import { IWebSocket } from "./foxglove-ws/types";
import { MessageData } from "./foxglove-ws/types";
import { parse } from "@foxglove/rosmsg";
import { MessageReader } from "@foxglove/rosmsg2-serialization";
import { StdMsgsString } from "./ros_interfaces";
import "./App.css";

function App() {
  const [greetMsg, setGreetMsg] = useState("");
  const [name, setName] = useState("");

  async function connect() {
    const ws = new WebSocket("ws://localhost:8765", [
      FoxgloveClient.SUPPORTED_SUBPROTOCOL,
    ]);
    const client = new FoxgloveClient({ ws: ws as IWebSocket });

    let chatterSubId: number;
    let chatterSchema: string = "";
    let chatterReader: MessageReader;
    client.on("open", () => {
      console.log("foxglove connected");
    });
    client.on("advertise", (newChannels) => {
      for (const channel of newChannels) {
        if (channel.topic === "/chatter") {
          chatterSchema = channel.schema;
          const chatterSchemaDefinition = parse(chatterSchema);
          chatterSubId = client.subscribe(channel.id);
          chatterReader = new MessageReader(chatterSchemaDefinition);
          console.log(`chatterSubId: ${chatterSubId}`);
          console.log(`${JSON.stringify(channel, null, 2)}`)
        }
      }
    });


    client.on("message", (event: MessageData) => {
      if (event.subscriptionId === chatterSubId) {
        const msg = chatterReader.readMessage(event.data) as StdMsgsString;
        console.log(`msg: ${JSON.stringify(msg, null, 2)}`);
        console.log(`chatter received: ${msg.data}`);
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
