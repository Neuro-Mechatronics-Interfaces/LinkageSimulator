export type RadialActionId = 'reposition' | 'add-link' | 'add-joint' | 'add-contactor' | 'delete' | 'cancel';

export interface RadialAction {
  id: RadialActionId;
  label: string;
  enabled: boolean;
}

export class RadialMenu {
  constructor(
    private readonly element: HTMLElement,
    private readonly onAction: (action: RadialActionId) => void,
  ) {}

  open(x: number, y: number, actions: RadialAction[]): void {
    this.element.replaceChildren();
    this.element.style.left = `${x}px`;
    this.element.style.top = `${y}px`;
    this.element.hidden = false;
    const radius = 82;
    actions.forEach((action, index) => {
      const angle = -Math.PI / 2 + (index * Math.PI * 2) / actions.length;
      const button = document.createElement('button');
      button.type = 'button';
      button.className = `radial-action radial-action-${action.id}`;
      button.textContent = action.label;
      button.disabled = !action.enabled;
      button.style.transform = `translate(-50%, -50%) translate(${Math.cos(angle) * radius}px, ${Math.sin(angle) * radius}px)`;
      button.addEventListener('pointerdown', (event) => event.stopPropagation());
      button.addEventListener('click', () => {
        this.close();
        if (action.enabled) this.onAction(action.id);
      });
      this.element.append(button);
    });
    const center = document.createElement('button');
    center.type = 'button';
    center.className = 'radial-center';
    center.textContent = '×';
    center.setAttribute('aria-label', 'Close menu');
    center.addEventListener('click', () => this.close());
    this.element.append(center);
  }

  close(): void {
    this.element.hidden = true;
    this.element.replaceChildren();
  }

  get isOpen(): boolean {
    return !this.element.hidden;
  }
}
