import {
  digitWidthAtContactor,
  ringAffordance,
  type AppStore,
  type Link,
  type RevoluteJoint,
  type SimulationState,
} from '../model';

const degrees = (radians: number): number => (radians * 180) / Math.PI;
const radians = (degreesValue: number): number => (degreesValue * Math.PI) / 180;

export class Inspector {
  constructor(
    private readonly content: HTMLElement,
    private readonly kindLabel: HTMLElement,
    private readonly onChange: () => void,
  ) {}

  render(store: AppStore): void {
    this.content.replaceChildren();
    const selection = store.selection;
    if (!selection) {
      this.kindLabel.textContent = 'Nothing selected';
      const empty = document.createElement('div');
      empty.className = 'empty-state';
      empty.innerHTML = '<div class="empty-icon">◎</div><p>Select a link, joint, servo, or contactor to inspect it.</p>';
      this.content.append(empty);
      this.renderSolverDiagnostics(store.state);
      return;
    }
    this.kindLabel.textContent = selection.kind;

    if (selection.kind === 'link' || selection.kind === 'link-end') {
      const link = store.state.links.find((candidate) => candidate.id === selection.id);
      if (link) {
        this.renderLink(link, store.state);
        if (selection.kind === 'link-end') this.addReadout('Handle', 'Right-end inverse control');
      }
    } else if (selection.kind === 'joint') {
      const joint = store.state.joints.find((candidate) => candidate.id === selection.id);
      if (joint) this.renderJoint(joint);
    } else if (selection.kind === 'servo') {
      this.renderServo(store.state);
    } else {
      const contactor = store.state.contactors.find((candidate) => candidate.id === selection.id);
      if (contactor) {
        this.addTitle(contactor.name, contactor.id);
        this.addReadout('Attached link', contactor.linkId);
        this.addReadout('Finger segment', contactor.fingerSegment);
        this.addReadout('Position', `${Math.round(contactor.fingerPosition * 100)}%`);
        this.addReadout('Contact geometry', 'Dorsal pad / flexor ring');
        const minimumRingWidth = digitWidthAtContactor(store.state.hand, contactor);
        this.addNumberField('Ring width', contactor.ringWidth, 'mm', minimumRingWidth, 100, 0.5, (value) => {
          contactor.ringWidth = Math.max(value, minimumRingWidth);
        });
        this.addReadout('Radial affordance', `${ringAffordance(store.state.hand, contactor).toFixed(1)} mm / side`);
        this.addReadout('Minimum ring width', `${minimumRingWidth.toFixed(1)} mm (digit width)`);
      }
    }
    this.renderSolverDiagnostics(store.state);
  }

  private renderSolverDiagnostics(state: SimulationState): void {
    const details = document.createElement('details');
    details.className = 'solver-diagnostics';
    const summary = document.createElement('summary');
    summary.textContent = 'Solver diagnostics';
    details.append(summary);
    for (const component of state.solverDiagnostics.components) {
      const row = document.createElement('div');
      row.className = 'solver-diagnostic-row';
      const status = component.inconsistent
        ? `residual ${component.residualNorm.toPrecision(3)}`
        : component.singular
          ? 'singular'
          : component.anchored
            ? 'anchored'
            : 'disconnected';
      row.textContent = `${component.id} · rank ${component.jacobianRank}/${component.variableCount} · DOF ${component.passiveDof}/${component.drivenDof} · ${status}`;
      row.title = component.messages.join('\n');
      details.append(row);
    }
    this.content.append(details);
  }

  private renderLink(link: Link, state: SimulationState): void {
    this.addTitle(link.name, link.id);
    const actuatorJoint = state.joints.find((joint) => joint.id === state.servo.revoluteJointId);
    const mountLinkId = actuatorJoint?.linkBId === state.servo.drivenLinkId
      ? actuatorJoint.linkAId
      : actuatorJoint?.linkBId;
    if (link.id === mountLinkId && link.fixed) {
      this.addReadout('Role', 'Servo-to-rocker dorsal mount');
      this.addNumberField('Rail length', link.length, 'mm', 15, 180, 1, (value) => { link.length = value; });
      this.addNumberField(
        'Angle from servo',
        degrees(state.ground.baseRailAngleOffset),
        '°',
        -120,
        120,
        1,
        (value) => { state.ground.baseRailAngleOffset = radians(value); },
      );
      this.addReadout('Reference', 'Primary servo axis');
      return;
    }
    if (link.fixed) {
      this.addReadout('Role', 'Fixed structural base');
      this.addReadout('Length', `${link.length.toFixed(1)} mm`);
      this.addReadout('Width', `${link.width.toFixed(1)} mm`);
      return;
    }
    this.addTextField('Name', link.name, (value) => { link.name = value || link.id; });
    this.addNumberField('Length', link.length, 'mm', 5, 250, 1, (value) => { link.length = value; });
    this.addNumberField('Width', link.width, 'mm', 2, 30, 0.5, (value) => { link.width = value; });
  }

  private renderJoint(joint: RevoluteJoint): void {
    this.addTitle(joint.name, joint.id);
    this.addReadout('Between', joint.linkAId ? `${joint.linkAId} / ${joint.linkBId}` : `ground / ${joint.linkBId}`);
    this.addNumberField('Minimum angle', degrees(joint.minAngle ?? -Math.PI), '°', -180, 180, 1, (value) => {
      joint.minAngle = radians(value);
    });
    this.addNumberField('Maximum angle', degrees(joint.maxAngle ?? Math.PI), '°', -180, 180, 1, (value) => {
      joint.maxAngle = radians(value);
    });
  }

  private renderServo(state: SimulationState): void {
    const servo = state.servo;
    this.addTitle(servo.name, servo.id);
    this.addReadout('Driven link', servo.drivenLinkId);
    this.addNumberField('Servo-ground offset', state.ground.servoGroundOffset, 'mm', 0, 120, 1, (value) => {
      state.ground.servoGroundOffset = value;
    });
    this.addReadout('Ground reference', state.ground.name);
    this.addNumberField('Minimum ROM', degrees(servo.minAngle), '°', -180, 179, 1, (value) => {
      servo.minAngle = Math.min(radians(value), servo.maxAngle - radians(1));
      servo.angle = Math.max(servo.angle, servo.minAngle);
    });
    this.addNumberField('Maximum ROM', degrees(servo.maxAngle), '°', -179, 180, 1, (value) => {
      servo.maxAngle = Math.max(radians(value), servo.minAngle + radians(1));
      servo.angle = Math.min(servo.angle, servo.maxAngle);
    });
    this.addNumberField('Sweep speed', degrees(servo.speed), '°/s', 1, 180, 1, (value) => {
      servo.speed = radians(value);
    });
  }

  private addTitle(name: string, id: string): void {
    const heading = document.createElement('div');
    heading.className = 'component-heading';
    const title = document.createElement('h2');
    title.textContent = name;
    const code = document.createElement('code');
    code.textContent = id;
    heading.append(title, code);
    this.content.append(heading);
  }

  private addReadout(label: string, value: string): void {
    const row = document.createElement('div');
    row.className = 'readout-row';
    const labelElement = document.createElement('span');
    labelElement.textContent = label;
    const valueElement = document.createElement('strong');
    valueElement.textContent = value;
    row.append(labelElement, valueElement);
    this.content.append(row);
  }

  private addTextField(label: string, value: string, update: (value: string) => void): void {
    const { row, input } = this.createField(label, 'text');
    input.value = value;
    input.addEventListener('input', () => {
      update(input.value);
      this.onChange();
    });
    this.content.append(row);
  }

  private addNumberField(
    label: string,
    value: number,
    unit: string,
    min: number,
    max: number,
    step: number,
    update: (value: number) => void,
  ): void {
    const { row, input, suffix } = this.createField(label, 'number');
    input.value = Number(value.toFixed(2)).toString();
    input.min = String(min);
    input.max = String(max);
    input.step = String(step);
    suffix.textContent = unit;
    input.addEventListener('change', () => {
      const parsed = Number(input.value);
      if (!Number.isFinite(parsed)) return;
      const clamped = Math.max(min, Math.min(max, parsed));
      input.value = String(clamped);
      update(clamped);
      this.onChange();
    });
    this.content.append(row);
  }

  private createField(label: string, type: 'text' | 'number'): {
    row: HTMLLabelElement;
    input: HTMLInputElement;
    suffix: HTMLSpanElement;
  } {
    const row = document.createElement('label');
    row.className = 'field-row';
    const labelElement = document.createElement('span');
    labelElement.textContent = label;
    const control = document.createElement('span');
    control.className = 'field-control';
    const input = document.createElement('input');
    input.type = type;
    const suffix = document.createElement('span');
    suffix.className = 'field-suffix';
    control.append(input, suffix);
    row.append(labelElement, control);
    return { row, input, suffix };
  }
}
