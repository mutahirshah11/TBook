import React, { useState, useEffect } from 'react';
import { 
  ChevronLeft, 
  ChevronRight, 
  MessageSquare, 
  Maximize2, 
  Minimize2,
  CheckCircle2,
  Settings2
} from 'lucide-react';
import { useHistory } from '@docusaurus/router';
import './FloatingControls.css';

export default function FloatingControls({ onZenToggle, isZen }: any) {
  const [scrolled, setScrolled] = useState(0);
  const history = useHistory();

  useEffect(() => {
    const handleScroll = () => {
      const winScroll = document.documentElement.scrollTop;
      const height = document.documentElement.scrollHeight - document.documentElement.clientHeight;
      const scrolled = (winScroll / height) * 100;
      setScrolled(scrolled);
    };
    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <>
      {/* Top Progress Bar */}
      <div className="reader-progress">
        <div className="reader-progress__bar" style={{ width: `${scrolled}%` }} />
      </div>

      {/* Bottom Floating Dock */}
      <div className="reader-dock-wrapper">
        <div className="reader-dock">
          <button className="reader-dock__item" onClick={() => window.history.back()} title="Go Back">
            <ChevronLeft size={20} />
          </button>
          
          <div className="reader-dock__divider" />
          
          <button className="reader-dock__item" onClick={onZenToggle} title={isZen ? "Exit Focus Mode" : "Focus Mode"}>
            {isZen ? <Minimize2 size={20} /> : <Maximize2 size={20} />}
          </button>
          
          <button className="reader-dock__item" title="Reader Settings">
            <Settings2 size={20} />
          </button>
          
          <div className="reader-dock__divider" />
          
          <button className="reader-dock__item reader-dock__item--success" title="Mark as Complete">
            <CheckCircle2 size={20} />
            <span className="reader-dock__label">Complete</span>
          </button>
        </div>
      </div>
    </>
  );
}
