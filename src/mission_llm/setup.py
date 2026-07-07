from setuptools import setup

package_name = 'mission_llm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # mission_schema.json and zones.json are read at runtime via
    # os.path.dirname(__file__), i.e. they must live inside the installed
    # site-packages/mission_llm directory alongside llm_node.py -- NOT
    # under share/. package_data is the correct mechanism for that.
    package_data={package_name: ['mission_schema.json', 'zones.json']},
    include_package_data=True,
    install_requires=['setuptools', 'jsonschema', 'anthropic'],
    zip_safe=True,
    maintainer='Pavan',
    maintainer_email='you@example.com',
    description='Prompt -> LLM -> validated mission JSON',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'llm_node = mission_llm.llm_node:main',
        ],
    },
)