import '../css/main.css';
import { createDefaultState, type AppStore } from './model';
import { CanvasRenderer } from './rendering';
import { MechanismSimulation } from './simulation';
import { CanvasInteraction, downloadSimulationJson, Inspector } from './ui';

function element<T extends HTMLElement>(id: string): T {
  const found = document.getElementById(id);
  if (!found) throw new Error(`Required element #${id} was not found.`);
  return found as T;
}

const canvas = element<HTMLCanvasElement>('simulation-canvas');
const shell = element<HTMLElement>('canvas-shell');
const playButton = element<HTMLButtonElement>('play-button');
const exportButton = element<HTMLButtonElement>('export-button');
const resetButton = element<HTMLButtonElement>('reset-button');
const servoAngle = element<HTMLInputElement>('servo-angle');
const servoAngleOutput = element<HTMLOutputElement>('servo-angle-output');
const handSize = element<HTMLInputElement>('hand-size');
const handSizeOutput = element<HTMLOutputElement>('hand-size-output');
const constructionToggle = element<HTMLInputElement>('construction-toggle');
const solverStatus = element<HTMLElement>('solver-status');
const mcpMomentOutput = element<HTMLOutputElement>('mcp-moment-output');
const pipMomentOutput = element<HTMLOutputElement>('pip-moment-output');
const dipMomentOutput = element<HTMLOutputElement>('dip-moment-output');
const digitMassOutput = element<HTMLOutputElement>('digit-mass-output');

const store: AppStore = { state: createDefaultState(), selection: null };
const simulation = new MechanismSimulation();
const renderer = new CanvasRenderer(canvas);
let inspector: Inspector;

const solveAndRefresh = (): void => {
  simulation.solve(store.state);
  syncControls();
  inspector.render(store);
};

inspector = new Inspector(
  element('inspector-content'),
  element('selection-kind'),
  solveAndRefresh,
);

const interaction = new CanvasInteraction(
  canvas,
  shell,
  element('radial-menu'),
  store,
  renderer,
  simulation,
  () => {
    inspector.render(store);
    syncControls();
  },
);

function degrees(radians: number): number {
  return (radians * 180) / Math.PI;
}

function syncControls(): void {
  const state = store.state;
  const angle = Math.round(degrees(state.servo.angle));
  servoAngle.min = String(Math.round(degrees(state.servo.minAngle)));
  servoAngle.max = String(Math.round(degrees(state.servo.maxAngle)));
  servoAngle.value = String(angle);
  servoAngleOutput.value = `${angle}°`;
  handSize.value = String(state.hand.sizeScale);
  handSizeOutput.value = `${Math.round(state.hand.sizeScale * 100)}%`;
  constructionToggle.checked = state.showConstruction;
  playButton.textContent = state.enabled ? 'Pause' : 'Play';
  playButton.classList.toggle('is-playing', state.enabled);
  solverStatus.textContent = state.message;
  solverStatus.classList.toggle('is-invalid', !state.valid);
  const moments = new Map(state.statics.jointMoments.map((moment) => [moment.jointId, moment]));
  const formatMoment = (jointId: 'mcp' | 'pip' | 'dip'): string => {
    const moment = moments.get(jointId)?.requiredHoldingMomentNm;
    return moment === undefined ? '—' : `${moment >= 0 ? '+' : ''}${(moment * 1000).toFixed(2)} mN·m`;
  };
  mcpMomentOutput.value = formatMoment('mcp');
  pipMomentOutput.value = formatMoment('pip');
  dipMomentOutput.value = formatMoment('dip');
  const digitMassG = state.statics.segmentMasses.reduce((sum, segment) => sum + segment.massKg, 0) * 1000;
  digitMassOutput.value = `${digitMassG.toFixed(1)} g`;
}

exportButton.addEventListener('click', () => {
  simulation.solve(store.state);
  downloadSimulationJson(store.state);
  syncControls();
});

playButton.addEventListener('click', () => {
  store.state.enabled = !store.state.enabled;
  syncControls();
});

resetButton.addEventListener('click', () => {
  store.state = createDefaultState();
  store.selection = null;
  interaction.closeMenu();
  renderer.camera.center = { x: -5, y: 35 };
  renderer.camera.zoom = 3.2;
  simulation.solve(store.state);
  inspector.render(store);
  syncControls();
});

servoAngle.addEventListener('input', () => {
  store.state.enabled = false;
  store.state.servo.angle = (Number(servoAngle.value) * Math.PI) / 180;
  simulation.solve(store.state);
  syncControls();
});

handSize.addEventListener('input', () => {
  store.state.hand.sizeScale = Number(handSize.value);
  simulation.solve(store.state);
  syncControls();
});

constructionToggle.addEventListener('change', () => {
  store.state.showConstruction = constructionToggle.checked;
});

simulation.solve(store.state);
inspector.render(store);
syncControls();

let previousTimestamp = performance.now();
function frame(timestamp: number): void {
  const dt = Math.min(0.05, Math.max(0, (timestamp - previousTimestamp) / 1000));
  previousTimestamp = timestamp;
  if (store.state.enabled) {
    simulation.step(store.state, dt);
    syncControls();
  }
  renderer.render(store.state, store.selection);
  requestAnimationFrame(frame);
}

requestAnimationFrame(frame);
