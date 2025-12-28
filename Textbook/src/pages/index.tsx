import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import { Cpu, Zap, Activity, Globe, Layers, Eye, BookOpen, ArrowRight, Star } from 'lucide-react';

import styles from './index.module.css';

function HomepageHeader() {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {  
    setIsVisible(true);
  }, []);

  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className={styles.heroVisual}>
        <div className={styles.heroOrb}></div>
        <div className={styles.heroGrid}></div>
        <div className={styles.heroFlow}></div>
      </div>
      
      <div className="container">
        <div className={clsx(styles.heroContent, isVisible && styles.visible)}>
          <div className={styles.heroBadge}>
              <Star size={12} style={{marginRight: '6px'}} fill="currentColor" />
              The Intelligence Layer for Robotics
          </div>
          <Heading as="h1" className={styles.mainTitle}>
            Physical AI &<br />Humanoid Robotics
          </Heading>
          <p className={styles.subtitle}>
            The most advanced engineering course for the next generation of humanoid developers. 
            From control systems to conversational embodied agents.
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/part1/foundations-physical-ai"
              style={{display:'flex', alignItems:'center', gap:'10px', padding: '1rem 2rem'}}>
              Start Learning <ArrowRight size={20} />
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/signup"
              style={{padding: '1rem 2rem'}}>
              Join Community
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function CourseStats() {
  const stats = [
    { value: '06', label: 'Modular Parts' },
    { value: '23', label: 'Advanced Chapters' },
    { value: '100+', label: 'Live Projects' },
  ];

  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className="row">
          {stats.map((stat, index) => (
            <div key={index} className="col col--4 text--center">
              <div className={styles.statValue}>{stat.value}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function TechnologiesSection() {
  const technologies = [
    { name: 'Embodied AI', desc: 'Neural architectures for physical systems and real-time inference.', icon: <Globe size={24} />, span: 'span 8' },
    { name: 'Computer Vision', desc: 'Spatial awareness and object tracking.', icon: <Layers size={24} />, span: 'span 4' },
    { name: 'Control Systems', desc: 'Precision motion and dynamic balance.', icon: <Cpu size={24} />, span: 'span 4' },
    { name: 'Locomotion', desc: 'Bipedal and quadrupedal movement algorithms.', icon: <Activity size={24} />, span: 'span 4' },
    { name: 'Sensors', desc: 'Multi-modal data fusion.', icon: <Eye size={24} />, span: 'span 4' },
  ];

  return (
    <section className={styles.sectionContainer}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Engineering Stack
          </Heading>
          <p className={styles.sectionSubtitle}>
            Master the underlying technologies driving the robotics revolution.
          </p>
        </div>
        
        <div className={styles.bentoGrid}>
          {technologies.map((tech, index) => (
            <div key={index} className={styles.bentoItem} style={{gridColumn: tech.span}}>
              <div className={styles.bentoItemIcon}>
                {tech.icon}
              </div>
              <h3 className={styles.bentoItemTitle}>{tech.name}</h3>
              <p className={styles.bentoItemDesc}>{tech.desc}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function UniqueValueProposition() {
  return (
    <section className={styles.sectionContainer} style={{background: 'rgba(255,255,255,0.01)'}}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            Why RoboLearn?
          </Heading>
        </div>
        
        <div className={styles.valueGrid}>
          <div className={styles.valueCard}>
            <div style={{color: '#6366f1', marginBottom: '1.5rem'}}><BookOpen size={32} /></div>
            <h3 className={styles.bentoItemTitle}>Research First</h3>
            <p className={styles.bentoItemDesc}>Direct implementation of papers from NVIDIA, Tesla, and Boston Dynamics labs.</p>
          </div>
          <div className={styles.valueCard}>
            <div style={{color: '#6366f1', marginBottom: '1.5rem'}}><Zap size={32} /></div>
            <h3 className={styles.bentoItemTitle}>Production Ready</h3>
            <p className={styles.bentoItemDesc}>Not just theory. Learn the deployment pipelines used in industry robotic fleets.</p>
          </div>
          <div className={styles.valueCard}>
            <div style={{color: '#6366f1', marginBottom: '1.5rem'}}><Cpu size={32} /></div>
            <h3 className={styles.bentoItemTitle}>AI-Integrated</h3>
            <p className={styles.bentoItemDesc}>Built-in RAG Assistant trained on the entire curriculum to support your labs 24/7.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  
  useEffect(() => {
    const handleScroll = () => {
      if (window.scrollY > 50) {
        document.body.classList.add('home-page--scrolled');
      } else {
        document.body.classList.remove('home-page--scrolled');
      }
    };
    
    document.body.classList.add('home-page');
    window.addEventListener('scroll', handleScroll);
    
    return () => {
      document.body.classList.remove('home-page', 'home-page--scrolled');
      window.removeEventListener('scroll', handleScroll);
    };
  }, []);

  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Professional engineering course for embodied intelligence.">
      <HomepageHeader />
      <main>
        <CourseStats />
        <TechnologiesSection />
        
        <div className="container">
          <div className={styles.courseOverview}>
            <h2 className={styles.overviewTitle}>Engineered for Mastery</h2>
            <p className={styles.overviewDescription}>
              A curriculum designed by robotics engineers for robotics engineers. 
              Move from URDF basics to transformer-based motion control in weeks.
            </p>
            <Link
              className="button button--primary button--lg"
              to="/docs/part1/foundations-physical-ai">
              Browse the Syllabus
            </Link>
          </div>
        </div>

        <UniqueValueProposition />
      </main>
    </Layout>
  );
}
