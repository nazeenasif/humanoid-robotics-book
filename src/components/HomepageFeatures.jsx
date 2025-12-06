import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI & Humanoid Robotics',
    description: (
      <>
        Comprehensive guide to building intelligent humanoid robots with modern AI techniques,
        covering ROS 2, digital twins, and vision-language-action systems.
      </>
    ),
  },
  {
    title: 'Simulation & Real-World Integration',
    description: (
      <>
        Learn to create digital twins using Gazebo and Unity, bridging simulation and
        real-world robot deployment with Isaac Sim and advanced perception systems.
      </>
    ),
  },
  {
    title: 'Advanced AI Integration',
    description: (
      <>
        Master AI-powered navigation, perception, and interaction using OpenAI GPT and Whisper
        for voice-controlled humanoid robot systems.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}