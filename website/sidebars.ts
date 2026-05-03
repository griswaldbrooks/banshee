import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  docsSidebar: [
    'intro',
    'banshee',
    {
      type: 'category',
      label: 'Build',
      items: ['build-flash', 'electronics', 'mechanicals', 'bom', 'onshape'],
    },
    {
      type: 'category',
      label: 'Seasons',
      items: ['seasons/index'],
    },
    'links',
  ],
};

export default sidebars;
