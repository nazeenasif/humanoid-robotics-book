import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <p className="hero__description">
          Discover the secrets of humanoid robotics and build intelligent machines with your own hands.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/quickstart">
            Start Reading →
          </Link>
        </div>
      </div>
    </header>
  );
}

function WhatThisBookCovers() {
  return (
    <section className={styles.whatThisBookCovers}>
      <div className="container padding-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <Heading as="h2" className="text--center margin-bottom--lg">
              What This Book Covers
            </Heading>
            <p className="text--center">
              This comprehensive guide takes you from ROS 2 fundamentals through advanced AI-powered humanoid robotics, covering everything from digital twin simulation to Vision-Language-Action systems. You'll master cutting-edge technologies like NVIDIA Isaac Sim, Gazebo physics simulation, Unity visualization, and OpenAI integration while building practical skills in humanoid robot control, perception, navigation, and multi-modal interaction. Whether you're a student, researcher, or developer, this book provides the knowledge and hands-on experience needed to create intelligent humanoid robots that can perceive, understand, and interact with the world around them.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Book">
      <HomepageHeader />
      <main>
        <WhatThisBookCovers />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}