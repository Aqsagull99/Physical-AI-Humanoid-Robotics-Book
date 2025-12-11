declare module '@theme/*' {
  import type { ComponentType } from 'react';
  const Component: ComponentType<any>;
  export default Component;
}
