from setuptools import find_packages, setup  # 导入查找包和打包入口函数

package_name = 'lekiwi_driver'  # 定义包名变量，后续复用

setup(  # 配置并声明当前 Python/ROS2 包信息
    name=package_name,  # 包名称
    version='0.0.0',  # 当前包版本
    packages=find_packages(exclude=['test']),  # 自动发现包，排除 test
    data_files=[  # 安装时需要拷贝的数据文件
        ('share/ament_index/resource_index/packages',  # ament 索引路径
            ['resource/' + package_name]),  # 注册 resource 标识文件
        ('share/' + package_name, ['package.xml']),  # 安装 package.xml 到 share
    ],
    install_requires=['setuptools'],  # 运行依赖
    zip_safe=True,  # 允许以 zip 形式安装
    maintainer='root',  # 维护者名称
    maintainer_email='root@todo.todo',  # 维护者邮箱
    description='TODO: Package description',  # 包描述（待完善）
    license='TODO: License declaration',  # 许可证声明（待完善）
    tests_require=['pytest'],  # 测试依赖
    entry_points={  # 定义可执行入口点
        'console_scripts': [  # 命令行脚本列表
            'base_driver = lekiwi_driver.driver_node:main',  # 注册 base_driver 命令
            'lekiwi_base_driver = lekiwi_driver.lekiwi_base_driver:main',  # 底盘驱动节点
            'lekiwi_arm_driver = lekiwi_driver.lekiwi_arm_driver:main',  # 机械臂驱动节点
            'lekiwi_brain_node = lekiwi_driver.lekiwi_brain_node:main',  # 大脑决策节点
        ],
    },
)  # 结束 setup 配置
