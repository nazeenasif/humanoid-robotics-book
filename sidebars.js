/**
 * Creating a sidebar enables you to: - Create an ordered group of docs
 * - Show a sidebar "slice" that only contains a subset of docs
 * - Home page, docs page, blog page, etc.
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: ['quickstart', 'content-guidelines'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 for Humanoid Control',
      items: [
        'module1-ros2/introduction',
        'module1-ros2/python-integration',
        'module1-ros2/humanoid-modeling',
        'module1-ros2/exercises-ros2',
        'module1-ros2/learning-outcomes',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins with Gazebo & Unity',
      items: [
        'module2-digital-twin/introduction',
        'module2-digital-twin/gazebo-simulation',
        'module2-digital-twin/unity-simulation',
        'module2-digital-twin/exercises-digital-twin',
        'module2-digital-twin/learning-outcomes',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI-Powered Perception & Navigation',
      items: [
        'module3-ai-robot-brain/introduction',
        'module3-ai-robot-brain/vslam',
        'module3-ai-robot-brain/nav2-ai-navigation',
        'module3-ai-robot-brain/reinforcement-learning',
        'module3-ai-robot-brain/exercises-ai-robot-brain',
        'module3-ai-robot-brain/learning-outcomes',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: LLM-VLA for Multi-modal Interaction',
      items: [
        'module4-vla/introduction',
        'module4-vla/vision-processing',
        'module4-vla/language-processing',
        'module4-vla/action-execution',
        'module4-vla/exercises-vla',
        'module4-vla/capstone-project',
        'module4-vla/learning-outcomes',
      ],
      collapsed: false,
    },
    {
      type: 'doc',
      id: 'glossary',
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

module.exports = sidebars;
