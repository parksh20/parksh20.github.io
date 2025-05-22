// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: '로보틱스 시뮬레이션 마스터 가이드',
  tagline: 'Isaac Sim, Isaac Lab, ROS 2 Humble 완벽 정복',
  favicon: 'img/favicon.ico', // Placeholder, will need to create this image

  // Set the production url of your site here
  url: 'https://your-domain.com', // Replace with actual domain
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/', // Adjust if deploying to a subfolder

  // GitHub pages deployment config.
  organizationName: 'your-org', // Replace with your GitHub org/user name.
  projectName: 'robotics-guide-kr', // Replace with your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'ko',
    locales: ['ko'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/robotics-guide-kr/tree/main/', // Adjust
          routeBasePath: '/docs', // Serve docs at /docs
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/robotics-guide-kr/tree/main/', // Adjust
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg', // Placeholder, will need to create
      navbar: {
        title: '로보틱스 가이드 KR',
        logo: {
          alt: '사이트 로고',
          src: 'img/logo.svg', // Placeholder, will need to create
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'isaacSimSidebar',
            position: 'left',
            label: 'Isaac Sim',
          },
          {
            type: 'docSidebar',
            sidebarId: 'isaacLabSidebar',
            position: 'left',
            label: 'Isaac Lab',
          },
          {
            type: 'docSidebar',
            sidebarId: 'ros2HumbleSidebar',
            position: 'left',
            label: 'ROS 2 Humble',
          },
          {to: '/blog', label: '블로그', position: 'left'},
          {
            to: '/about', // Link to the About page
            label: '소개',
            position: 'right',
          },
          {
            href: 'https://github.com/your-org/robotics-guide-kr', // Adjust
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: '가이드',
            items: [
              {
                label: 'Isaac Sim',
                to: '/docs/isaac_sim/overview',
              },
              {
                label: 'Isaac Lab',
                to: '/docs/isaac_lab/introduction_to_isaac_lab',
              },
              {
                label: 'ROS 2 Humble',
                to: '/docs/ros2_humble/ros2_basics_refresh',
              },
            ],
          },
          {
            title: '커뮤니티',
            items: [
              {
                label: 'Stack Overflow (Isaac Sim)',
                href: 'https://stackoverflow.com/questions/tagged/nvidia-isaac',
              },
              {
                label: 'NVIDIA Developer Forums (Isaac Sim)',
                href: 'https://forums.developer.nvidia.com/c/omniverse/simulation/isaac-sim/3 Isaac Sim',
              },
              {
                label: 'ROS Discourse (ROS 2)',
                href: 'https://discourse.ros.org/',
              },
            ],
          },
          {
            title: '더 보기',
            items: [
              {
                label: '블로그',
                to: '/blog',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/robotics-guide-kr', // Adjust
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} 로보틱스 가이드 KR. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
      // Algolia DocSearch config (optional)
      // algolia: {
      //   appId: 'YOUR_APP_ID',
      //   apiKey: 'YOUR_SEARCH_API_KEY',
      //   indexName: 'YOUR_INDEX_NAME',
      //   contextualSearch: true,
      // },
    }),
};

module.exports = config;
