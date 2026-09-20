import { createRoot } from "react-dom/client";
import App from "./App";
import { lazy, Suspense } from "react";
import "@fontsource/inter/latin-400.css";
import "@fontsource/inter/latin-500.css";
import "@fontsource/inter/latin-600.css";
import "@fontsource/barlow-condensed/latin-500.css";
import "@fontsource/ibm-plex-mono/latin-400.css";
import "./style.css";
const DriveLab = lazy(() => import("./drive/DriveLab"));
const ContactLab = lazy(() => import("./contact/ContactLab"));
createRoot(document.getElementById("root")!).render(
  new URLSearchParams(location.search).has("drive") ? (
    <Suspense fallback={<p>Connecting to native simulator…</p>}>
      <DriveLab />
    </Suspense>
  ) : new URLSearchParams(location.search).has("contact") ? (
    <Suspense fallback={<p>Loading contact lab…</p>}>
      <ContactLab />
    </Suspense>
  ) : (
    <App />
  ),
);
