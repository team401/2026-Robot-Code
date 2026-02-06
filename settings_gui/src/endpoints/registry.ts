import type { ComponentType } from 'react';
import { ShotMapsEditor } from '../pages/ShotMapsEditor';

export interface EndpointEntry {
  name: string;
  label: string;
  component: ComponentType;
}

export const endpoints: EndpointEntry[] = [
  { name: 'shotmaps', label: 'Shot Maps', component: ShotMapsEditor },
];
