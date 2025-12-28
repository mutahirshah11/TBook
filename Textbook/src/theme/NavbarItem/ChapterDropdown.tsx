import React from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import clsx from 'clsx';

const CHAPTERS = [
  { label: 'Introduction', value: '/docs/intro' },
  { label: 'Part 1: Foundations', value: '/docs/part1/foundations-physical-ai' },
  { label: 'Ch 2: Embodied AI', value: '/docs/part1/chapter2-embodied-ai' },
  { label: 'Ch 3: Humanoid Landscape', value: '/docs/part1/chapter3-humanoid-landscape' },
  { label: 'Ch 4: Sensor Systems', value: '/docs/part1/chapter4-sensor-systems' },
  { label: 'Part 2: Embodied AI & ROS 2', value: '/docs/part2/chapter-5-ros2-architecture-and-core-concepts' },
  { label: 'Ch 6: Nodes, Topics', value: '/docs/part2/chapter-6-nodes-topics-services-actions' },
  { label: 'Ch 7: Building Packages', value: '/docs/part2/chapter-7-building-packages' },
  { label: 'Ch 8: Launch Files', value: '/docs/part2/chapter_8' },
  { label: 'Part 3: Gazebo Sim', value: '/docs/part3/chapter_9' },
  { label: 'Ch 10: URDF/SDF', value: '/docs/part3/chapter_10' },
  { label: 'Ch 11: Physics & Sensors', value: '/docs/part3/chapter_11' },
  { label: 'Ch 12: Unity Viz', value: '/docs/part3/chapter-12-unity-viz' },
  { label: 'Part 4: NVIDIA Isaac', value: '/docs/part4/chapter-13-isaac-intro' },
  { label: 'Ch 14: Perception', value: '/docs/part4/chapter-14-perception' },
  { label: 'Ch 15: RL Control', value: '/docs/part4/chapter-15-rl-control' },
  { label: 'Ch 16: Sim to Real', value: '/docs/part4/chapter-16-sim-to-real' },
  { label: 'Part 5: Humanoid Dev', value: '/docs/part5/chapter-17-kinematics-dynamics' },
  { label: 'Ch 18: Locomotion', value: '/docs/part5/chapter-18-locomotion' },
  { label: 'Ch 19: Manipulation', value: '/docs/part5/chapter-19-manipulation' },
  { label: 'Ch 20: HRI', value: '/docs/part5/chapter-20-hri' },
  { label: 'Part 6: Conversational', value: '/docs/part6/chapter-21-llm-integration' },
  { label: 'Ch 22: Speech & NLU', value: '/docs/part6/chapter-22-speech-nlu' },
  { label: 'Ch 23: Multimodal', value: '/docs/part6/chapter-23-multimodal' },
];

export default function ChapterDropdown(props: any) {
  const history = useHistory();
  const location = useLocation();
  const currentPath = location.pathname;

  // Find the closest matching chapter or default to first
  const currentChapter = CHAPTERS.find(c => currentPath.includes(c.value))?.value || '';

  const handleChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const val = e.target.value;
    if (val) history.push(val);
  };

  return (
    <div className={clsx('navbar__item', props.className)} style={{ display: 'flex', alignItems: 'center' }}>
      <select 
        value={currentChapter} 
        onChange={handleChange}
        style={{
            maxWidth: '180px',
            padding: '6px 12px',
            borderRadius: '8px',
            border: '1px solid rgba(255,255,255,0.2)',
            background: 'rgba(255,255,255,0.05)',
            color: '#fff',
            fontSize: '0.85rem',
            cursor: 'pointer',
            outline: 'none',
            // Truncate text
            textOverflow: 'ellipsis',
            whiteSpace: 'nowrap',
            overflow: 'hidden'
        }}
      >
        <option value="" disabled>Jump to Chapter...</option>
        {CHAPTERS.map((chapter) => (
          <option key={chapter.value} value={chapter.value} style={{color: '#000'}}>
            {chapter.label}
          </option>
        ))}
      </select>
    </div>
  );
}
