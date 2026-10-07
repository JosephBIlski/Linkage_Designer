import './style.css';
import { App } from './app';
import { UI } from './ui/panels';
import { ToolManager } from './viewport/tools';

const root = document.getElementById('app')!;
const viewportEl = document.getElementById('viewport')!;
const app = new App(viewportEl);
const tools = new ToolManager(app);
const ui = new UI(app, tools, root);
tools.reset();

// Start with the four-bar example so the first screen is not empty.
app.loadExample('fourBar');

// expose for debugging in the browser console
(window as unknown as { linkageDesigner: unknown }).linkageDesigner = { app, tools, ui };
