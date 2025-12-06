import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manually defined sidebar to control order and hierarchy
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Part 1: Foundations of Physical AI',
      items: [
        'part1/foundations-physical-ai',
        'part1/chapter2-embodied-ai',
        'part1/chapter3-humanoid-landscape',
        'part1/chapter4-sensor-systems',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: Embodied AI & ROS 2 Foundations',
      items: [
        'part2/chapter-5-ros2-architecture-and-core-concepts',
        'part2/chapter-6-nodes-topics-services-actions',
        'part2/chapter-7-building-packages',
      ],
    },
  ],
};

export default sidebars;
