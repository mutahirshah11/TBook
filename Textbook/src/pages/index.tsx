import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {  
    setIsVisible(true);
  }, []);

  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={clsx(styles.heroContent, isVisible && styles.visible)}>
          <div className={styles.heroText}>
            <div className={styles.heroBadge}>PREMIUM COURSE</div>
            <Heading as="h1" className={clsx('hero__title', styles.mainTitle)}>
              Physical AI &<br />Humanoid Robotics
            </Heading>
            <p className={clsx('hero__subtitle', styles.subtitle)}>
              Master the cutting-edge intersection of artificial intelligence and robotics engineering.
              Explore embodied intelligence, humanoid locomotion, and conversational AI systems.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/part1/foundations-physical-ai">
                Start Learning
              </Link>
              <Link
                className="button button--outline button--lg"
                to="/docs/part1/foundations-physical-ai">
                View Curriculum
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <div className={styles.heroOrb}></div>
            <div className={styles.heroGrid}></div>
            <div className={styles.heroGraphic}>
              <div className={styles.humanoidSilhouette}></div>
              <div className={styles.aiParticles}>
                {[...Array(12)].map((_, i) => (
                  <div key={i} className={styles.aiParticle} style={{'--delay': `${i * 0.2}s`}}></div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function StatCard({ number, label, description }) {
  return (
    <div className={clsx('card', styles.statCard)}>
      <div className={styles.statNumber}>{number}</div>
      <div className={styles.statLabel}>{label}</div>
      <div className={styles.statDescription}>{description}</div>
    </div>
  );
}

function FeatureCard({ title, description, icon, delay }) {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => setIsVisible(true), delay || 0);
    return () => clearTimeout(timer);
  }, [delay]);

  return (
    <div className={clsx('card', styles.featureCard, isVisible && styles.visible)}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function TechnologiesSection() {
  const technologies = [
    { name: 'Sensors', desc: 'Advanced perception systems' },
    { name: 'Actuators', desc: 'Precision motion control' },
    { name: 'Locomotion', desc: 'Bipedal movement algorithms' },
    { name: 'Control Systems', desc: 'Real-time dynamics' },
    { name: 'Embodied AI', desc: 'Physical intelligence' },
    { name: 'Computer Vision', desc: 'Visual perception' },
  ];

  return (
    <section className={styles.technologiesSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles.sectionTitle}>
              Core Technologies
            </Heading>
            <p className={styles.sectionSubtitle}>
              Master the cutting-edge tools and frameworks used in modern robotics
            </p>
          </div>
        </div>
        <div className="row">
          {technologies.map((tech, index) => (
            <div key={index} className="col col--4">
              <div className={styles.techItem}>
                <h3 className={styles.techTitle}>{tech.name}</h3>
                <p className={styles.techDescription}>{tech.desc}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function UniqueValueProposition() {
  return (
    <section className={styles.valueSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={styles.sectionTitle}>
              Why This Course Stands Apart
            </Heading>
          </div>
        </div>
        <div className="row">
          <div className="col col--4">
            <div className={styles.valueItem}>
              <div className={styles.valueIcon}>🔬</div>
              <h3 className={styles.valueTitle}>Research-Backed</h3>
              <p className={styles.valueDescription}>
                Based on the latest findings from top robotics labs and research institutions.
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.valueItem}>
              <div className={styles.valueIcon}>🛠️</div>
              <h3 className={styles.valueTitle}>Hands-On Projects</h3>
              <p className={styles.valueDescription}>
                Build and deploy real robotic systems with guided, practical exercises.
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.valueItem}>
              <div className={styles.valueIcon}>🚀</div>
              <h3 className={styles.valueTitle}>Industry Ready</h3>
              <p className={styles.valueDescription}>
                Learn skills directly applicable to careers in robotics and AI companies.
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function CourseStats() {
  const stats = [
    { number: '6', label: 'Parts', description: 'Comprehensive curriculum' },
    { number: '23', label: 'Chapters', description: 'In-depth coverage' },
    { number: '100+', label: 'Examples', description: 'Practical applications' },
  ];

  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className="row">
          {stats.map((stat, index) => (
            <div key={index} className="col col--4">
              <StatCard
                number={stat.number}
                label={stat.label}
                description={stat.description}
              />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="A comprehensive course on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <CourseStats />
        <div className="container">
          <div className="row">
            <div className="col col--12">
              <div className={clsx('glass-panel', styles.courseOverview)}>
                <h2 className={styles.overviewTitle}>Complete Course Overview</h2>
                <p className={styles.overviewDescription}>
                  Dive deep into six comprehensive parts covering everything from foundational concepts
                  to advanced conversational robotics systems. Each section builds upon the previous,
                  creating a cohesive learning journey.
                </p>
                <Link
                  className="button button--primary button--lg"
                  to="/docs/part1/foundations-physical-ai">
                  Explore Full Curriculum
                </Link>
              </div>
            </div>
          </div>
        </div>
        <TechnologiesSection />
        <UniqueValueProposition />
      </main>
    </Layout>
  );
}