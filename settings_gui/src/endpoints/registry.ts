import type { ComponentType } from 'react';
import { ShotMapsEditor } from '../pages/ShotMapsEditor';
import { VisionEditor } from '../pages/VisionEditor';

export interface EndpointEntry {
  name: string;
  label: string;
  component: ComponentType;
}

export const endpoints: EndpointEntry[] = [
  { name: 'shotmaps', label: 'Shot Maps', component: ShotMapsEditor },
  { name: 'vision', label: 'Vision', component: VisionEditor },
];
