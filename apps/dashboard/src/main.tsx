import { createRoot } from "react-dom/client";
import App from "./App";
import "@fontsource/inter/latin-400.css";
import "@fontsource/inter/latin-500.css";
import "@fontsource/inter/latin-600.css";
import "@fontsource/barlow-condensed/latin-500.css";
import "@fontsource/ibm-plex-mono/latin-400.css";
import "./style.css";
createRoot(document.getElementById("root")!).render(<App />);
