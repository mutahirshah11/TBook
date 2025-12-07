import React, { useState } from 'react';
import styles from './styles.module.css';

export default function EmbodimentSlider() {
  const [value, setValue] = useState(50);

  const getDescription = (val: number) => {
    if (val < 30) return "Pure Software: The brain in the jar. Intelligence is entirely algorithmic. Hardware is just a container.";
    if (val < 70) return "Coupled Control: The software drives the hardware, but they are distinct. The body is a puppet.";
    return "Deep Embodiment: The body IS the mind. Morphological computation (e.g., passive walking) handles tasks without CPU cycles.";
  };

  const getLabel = (val: number) => {
    if (val < 30) return "Disembodied (Digital AI)";
    if (val < 70) return "Mechanically Actuated";
    return "Fully Embodied (Physical AI)";
  };

  return (
    <div className="p-4 border rounded-lg bg-gray-50 dark:bg-gray-800 my-4">
      <h3 className="text-lg font-bold mb-2">Spectrum of Embodiment</h3>
      <div className="flex flex-col gap-4">
        <input
          type="range"
          min="0"
          max="100"
          value={value}
          onChange={(e) => setValue(parseInt(e.target.value))}
          className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer dark:bg-gray-700"
        />
        <div className="flex justify-between text-xs text-gray-500">
          <span>Software</span>
          <span>Hardware</span>
        </div>
        
        <div className="bg-white dark:bg-gray-900 p-4 rounded shadow-sm border-l-4 border-blue-500">
          <h4 className="font-bold text-blue-600">{getLabel(value)}</h4>
          <p className="mt-1 text-sm text-gray-600 dark:text-gray-300">{getDescription(value)}</p>
        </div>
      </div>
    </div>
  );
}
