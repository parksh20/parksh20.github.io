// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Sidebar for Isaac Sim
  isaacSimSidebar: [
    {
      type: 'category',
      label: 'Isaac Sim 시작하기',
      link: {
        type: 'doc',
        id: 'isaac_sim/overview', // Link to the overview page
      },
      items: [
        'isaac_sim/overview',
        // 'isaac_sim/installation_guide', // Removed as it's covered in beginner_tutorials
        {
          type: 'category',
          label: '초급 튜토리얼',
          link: {
            type: 'generated-index',
            title: 'Isaac Sim 초급 튜토리얼',
            description: 'Isaac Sim 사용을 위한 기본 튜토리얼입니다.',
            slug: '/docs/isaac_sim/beginner_tutorials',
          },
          items: [
            'isaac_sim/beginner_tutorials/1_introduction_to_isaac_sim',
            'isaac_sim/beginner_tutorials/2_installation_guide',
            'isaac_sim/beginner_tutorials/3_getting_started_ui_and_scene_setup',
            'isaac_sim/beginner_tutorials/4_working_with_prims',
            'isaac_sim/beginner_tutorials/5_physics_essentials',
            'isaac_sim/beginner_tutorials/6_importing_assets',
          ],
        },
        // Placeholder for Intermediate Tutorials (can be expanded later)
        // {
        //   type: 'category',
        //   label: '중급 튜토리얼',
        //   items: ['isaac_sim/intermediate_tutorials/placeholder'],
        // },
        // Placeholder for Advanced Tutorials (can be expanded later)
        // {
        //   type: 'category',
        //   label: '고급 튜토리얼',
        //   items: ['isaac_sim/advanced_tutorials/placeholder'],
        // },
        'isaac_sim/project_showcases', // Assuming this is a single doc
      ],
    },
    {
      type: 'category',
      label: 'Isaac Sim 고급 토픽',
      link: {
        type: 'generated-index',
        title: 'Isaac Sim 고급 토픽',
        description: 'Isaac Sim의 고급 기능 및 활용법을 다룹니다.',
        slug: '/docs/isaac_sim/advanced_topics_index', // Placeholder slug
      },
      items: [
        'advanced_topics/isaac_sim_advanced/1_omnigraph_deep_dive',
        'advanced_topics/isaac_sim_advanced/2_custom_extensions',
        'advanced_topics/isaac_sim_advanced/3_performance_tuning',
        'advanced_topics/isaac_sim_advanced/4_external_applications_integration',
        'advanced_topics/isaac_sim_advanced/5_advanced_rendering',
      ],
    }
  ],

  // Sidebar for Isaac Lab
  isaacLabSidebar: [
    {
      type: 'category',
      label: 'Isaac Lab 시작하기',
      link: {
        type: 'doc',
        id: 'isaac_lab/introduction_to_isaac_lab', // Link to Isaac Lab intro
      },
      items: [
        'isaac_lab/introduction_to_isaac_lab',
        'isaac_lab/installation_guide',
        'isaac_lab/core_concepts',
        // Placeholder for Tutorials (can be expanded later)
        // {
        //   type: 'category',
        //   label: '튜토리얼',
        //   items: ['isaac_lab/tutorials/placeholder'],
        // },
      ],
    },
    {
      type: 'category',
      label: 'Isaac Lab 고급 토픽',
      link: {
        type: 'generated-index',
        title: 'Isaac Lab 고급 토픽',
        description: 'Isaac Lab의 고급 활용법과 심층 분석을 다룹니다.',
        slug: '/docs/isaac_lab/advanced_topics_index', // Placeholder slug
      },
      items: [
        'advanced_topics/isaac_lab_advanced/1_domain_randomization',
        'advanced_topics/isaac_lab_advanced/2_curriculum_learning',
        'advanced_topics/isaac_lab_advanced/3_custom_rl_agents',
        'advanced_topics/isaac_lab_advanced/4_multi_robot_scenarios',
        'advanced_topics/isaac_lab_advanced/5_simulation_data_logging_analysis',
      ],
    }
  ],

  // Sidebar for ROS 2 Humble
  ros2HumbleSidebar: [
    {
      type: 'category',
      label: 'ROS 2 Humble',
      link: {
        type: 'doc',
        id: 'ros2_humble/ros2_basics_refresh', // Link to ROS 2 Basics
      },
      items: [
        'ros2_humble/ros2_basics_refresh',
        'ros2_humble/isaac_sim_ros2_integration',
        'ros2_humble/isaac_lab_ros2_integration',
        // Placeholder for Tutorials (can be expanded later)
        // {
        //   type: 'category',
        //   label: 'ROS 2 튜토리얼',
        //   items: ['ros2_humble/tutorials/placeholder'],
        // },
      ],
    },
  ],
};

module.exports = sidebars;
