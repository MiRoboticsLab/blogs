**可视化编程-协议及接口文档**

**概述**

当前协议为可视化编程唯一合法协议，协议属于python3语法的子集，故本协议一二三节内支持的任何内容有任何不清楚的都可通过python书籍查阅。

**零、交互约束**

**0.1 交互时ros/grpc格式**

术语约束：

> ‘前端’：代指APP及WEB可视化编程控制端；
> 
> ‘机器人端’：代指机器人侧可视化编程引擎。

**0.1.1 GRPC协议简介**

> 关于grpc详情参见附件3。

**0.1.2 机器人端监听GRPC消息约束**

消息类型：std\_msgs/msg/string

消息名称：/frontend\_message

消息内容：要符合下一节交互时json格式约束

其他约束：

<table>
<tbody>
<tr class="odd">
<td>Apache<br />
VISUAL_FRONTEND_MSG = 2002;</td>
</tr>
</tbody>
</table>

**0.1.3 GRPC监听机器人端消息约束**

消息类型：std\_msgs/msg/string

消息名称：/backend\_message

消息内容：要符合下一节交互时json格式约束

其他约束：

<table>
<tbody>
<tr class="odd">
<td>Apache<br />
VISUAL_BACKEND_MSG = 2001;</td>
</tr>
</tbody>
</table>

**0.2 任务及模块功能：**

**0.2.1 \<前端-机器人端\>/\<机器人端-后端\>交互时json格式**

**0.2.1.1 \[前端-\>机器人端\]/\[机器人端-\>后端\]：下发数据json格式**

**0.2.1.1.1 协议约束**

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
{<br />
"type": "type",<br />
"id": "id",<br />
"operate": "operate",<br />
"target_id": [<br />
"target_id_1",<br />
"target_id_2",<br />
"target_id_3",<br />
...<br />
],<br />
"describe": "describe",<br />
"style": "style",<br />
"mode": "mode",<br />
"condition": "condition",<br />
"body": "body"<br />
}</td>
</tr>
</tbody>
</table>

**0.2.1.1.2 协议说明**

**0.2.1.1.2.1 通用字段**

**type:** \[必须\] 字符串类型，标识当前json数据类型。

合法参数：

任务:'task'

模块:'module'

模块:'AI'

模块:‘SLAM’

**id**: \[必须\] 字符串类型，标识当前数据帧的唯一id，用于反馈对齐；

合法参数要求：符合变量命名规则的唯一标识当前数据帧的id。

**operate**: \[必须\] 字符串类型，标识当前任务操作类型。

仅当'type'字段值为'task'或' module'时该字段有效且必须。

合法参数：

保存任务/模块:'save'

删除任务/模块:'delete'

查询任务/模块/AI:'inquiry'

调试任务:'debug'

运行任务:'run'

终止任务:'shutdown'

暂停任务:'suspend'

继续任务:'recover'

**target\_id**: \[必须\] 字符串数组类型，用于标识当前操作目标，根据场景可分为任务id、模块id。

合法参数要求：符合变量命名规则的唯一标识当前数据帧的id。

当operate字段为'debug'时候，target\_id固定为\["debug"\]。

任务id，当用于保存、删除、查询、调试、运行、暂停、继续、终止任务时，值为要操作的任务id；

模块id，当用于保存、删除、查询及调试模块时，值为要操作的模块id；

注意：仅当查询场景时，该字段允许为空，详情如下：

保存、运行场景时，若**target\_id**数组为空**，**则不执行，反之仅操作首个target\_id约束的目标（也就是说仅第一个id有效）；

运行任务时，会根据保存的任务属性进行运行；

暂停、继续、终止任务场景时，若**target\_id**数组为空**，**则不执行，反之仅执行target\_id约束的所有目标；

删除场景时，若**target\_id**数组为空**，**则不执行删除操作，反之仅删除target\_id约束的所有目标；

查询场景时，若**target\_id**数组为空**，**则返回type约束的所有目标，反之仅返回target\_id约束的所有目标；

**describe**: \[可选\] 字符串类型，描述当前数据帧功能，比如任务或模块场景。

仅当'operate'字段值为‘save’或'debug'时该字段有效且必须。

合法参数要求：不可包含三个连续的双引号(""")。

**style**: \[可选\] 字符串类型，标识当前消息样式，即当前执行主体内容的呈现方式。

仅当'operate'字段值为'save'或'debug'时该字段有效且必须。

**0.2.1.1.2.2 差异字段**

**0.2.1.1.2.2.1 type=task**

**mode**: \[可选\] 字符串类型，标识当前任务模式类型。

当'operate'字段值为'debug'时该字段固定为'single'

当'operate'字段值为'save'时该字段有效且必须

当'operate'字段值为'run'时该字段可选，如果赋值将修改原始对应字段属性；

合法参数：

单次任务：'single'

周期任务：'cycle'

**condition**: \[可选\] 当前帧为操作任务时，标识任务约束，详情如下：

> 符合下述约束的字符串类型，标识当前任务执行的先决条件。

当'operate'字段值为'debug'时该字段固定为'now'；

当'mode'字段值非空时该字段有效且必须，用于标识任务的默认约束；

当'operate'字段值为'save'或‘debug’时该字段标识当前任务的默认执行约束条件；

当'operate'字段值为'run'时该字段将修改默认的执行约束条件；

赋值方式有如下两种，第一种用于单次任务（mode = single），第二种用于周期任务（mode = cycle），格式分别如下：

第一种：单次任务（mode = single）

可采用绝对时间或相对时间格式进行约束，如果时间已过，则它会在第二天的同一时间执行。

格式分别如下：

绝对时间格式：

**'now'**：采用当前时间为约束时间，即立刻执行。

**HH:MM**：采用“小时:分钟”形式指定约束时间。

**HH:MM YYYY-MM-DD**：采用“小时:分钟 年-月-日”形式指定约束时间。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
// <strong>举例</strong><br />
{...,<br />
"condition": "now", //1. 立即执行当前任务<br />
"condition": "16:50", //2. 当天16:50执行当前任务<br />
"condition": "00:01 2022-12-01", //3. 在2022年12月1日00:01执行当前任务<br />
...}</td>
</tr>
</tbody>
</table>

相对时间格式：

**绝对时间 + number\[minutes|hours|days|weeks|months|years\]:**采用“绝对时间 + n\[分|时|天|周|月|年\]”形式指定约束时间，根据绝对时间格式可分为如下三种：

**now + number\[minutes|hours|days|weeks|months|years\]**

**HH:MM + number\[minutes|hours|days|weeks|months|years\]**

**HH:MM YYYY-MM-DD + number\[minutes|hours|days|weeks|months|years\]**

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
// <strong>举例</strong><br />
// "condition": <strong>绝对时间 + number[minutes|hours|days|weeks|months|years]</strong><br />
{...,<br />
"condition": "now + 5minutes", // 1. 5分钟后执行当前任务<br />
"condition": "16:50 + 5days", // 2. 5天后的16:50执行当前任务<br />
"condition": "00:01 2022-12-01 + 5years", // 3. 相对与2022年12月1日00:01而言，5年后执行当前任务<br />
...}</td>
</tr>
</tbody>
</table>

第二种：周期任务（mode = cycle）

> 采用格式如下：

**minute hour day month week**

其中：

**minute**：分钟 (0 - 59)

**hour**：小时 (0 - 23)

**day**：一个月中的第几天(1 - 31)

**month**：月份 (1 - 12)

**week**：周几 (0 - 6)

备注：

**\*** 取值范围内的所有数字

**/** 每过多少个数字

**-** 从X到Z

**，**散列数字

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
// 举例<br />
// <strong>minute hour day month week</strong><br />
{...,<br />
"condition": "* * * * *", // 1. 每分钟执行一次当前任务<br />
"condition": "*/1 * * * *", // 2. 每秒钟执行一次当前任务<br />
"condition": "01 * * * *", // 3. 每小时执行一次当前任务<br />
"condition": "3,15 * * * *", // 4. 每天每小时的第3和第15分钟执行一次当前任务<br />
"condition": "3,15 8-11 * * *", // 5. 每天上午8点到11点的第3和第15分钟执行<br />
"condition": "3,15 8-11 */2 * *", // 6. 每隔两天的上午8点到11点的第3和第15分钟执行一次当前任务<br />
"condition": "3,15 8-11 * * 1", // 7. 每个星期一的上午8点到11点的第3和第15分钟执行一次当前任务<br />
"condition": "30 21 * * *", // 8. 每晚的21:30执行一次当前任务<br />
"condition": "45 4 1,10,22 * *", // 9. 每月1、10、22日的4:45执行一次当前任务<br />
"condition": "0,30 18-23 * * *", // 10. 每天18:00至23:00之间每隔30分钟执行一次当前任务<br />
...}</td>
</tr>
</tbody>
</table>

单独动作

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
// 举例<br />
{...,<br />
"condition": "@reboot", // 1. 每次启动时运行一次当前任务<br />
...}</td>
</tr>
</tbody>
</table>

**body**: \[可选\] 字符串类型，标识当前消息正文，即当前执行的主体内容。

仅当'operate'字段值为'save'或'debug'时该字段有效且必须

数据字段缩进要求最小单位为4个空格。

**0.2.1.1.2.2.2 type=module**

**mode**: \[可选\] 字符串类型，标识当前模块模式类型。

仅当'operate'字段值为'add'时该字段有效且必须

合法参数：

普通模块：'common'

普通逻辑模块；

序列模块：'sequence'

用于设置动作序列的模块。

**condition**: \[可选\] 字符串类型，标识当前模块接口。

接口格式约束："函数名(参数)"。

仅当'operate'字段值为'add'时该字段有效且必须。

要求符合变量命名规则，注意不能和现有模块名称冲突。

**body**: \[可选\] 字符串类型，标识当前模块正文，即当前执行的主体内容。

仅当'operate'字段值为'add'时该字段有效且必须

数据字段缩进要求最小单位为4个空格，后期缩进为4的整数倍递增。

备注：模块不支持重载。

**0.2.1.1.2.2.3 type=AI**

**mode**: \[可选\] 字符串类型，标识当前模块模式类型。

仅当'operate'字段值为'inquiry'时该字段有效且必须

合法参数：

所有AI数据:'all'

人员:'personnel'

人脸:'face'

声纹:'voiceprint'

训练词:'training\_words'

**0.2.1.1.2.2.4 type=SLAM**

**mode**: \[可选\] 字符串类型，标识当前模块模式类型。

仅当'operate'字段值为'inquiry'时该字段有效且必须

合法参数：

地图:'map'，当前机器人所有地图信息。

预置点:'preset'，当前机器人所在地图中预先设置好的点，可以直接用来导航。

**0.2.1.1.3 协议举例**

**0.2.1.1.3.1 task 协议举例**

**0.2.1.1.3.1.1 调试代码**

先站立，等待60秒钟，再趴下。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["debug"],</strong><br />
<strong>"operate": "debug",</strong><br />
<strong>"mode": "single",</strong><br />
<strong>"condition": "now",</strong><br />
<strong>"body": "</strong><br />
<strong>cyberdog.motion.stand_up()</strong><br />
<strong>time.sleep(60)</strong><br />
<strong>cyberdog.motion.get_down()</strong><br />
<strong>"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.1.2 保存任务**

保存一条立即执行的任务：先站立，等待60秒钟，再趴下。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "save",</strong><br />
<strong>"mode": "single",</strong><br />
<strong>"condition": "now",</strong><br />
<strong>"body": "</strong><br />
<strong>cyberdog.motion.stand_up()</strong><br />
<strong>time.sleep(60)</strong><br />
<strong>cyberdog.motion.get_down()</strong><br />
<strong>"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

保存一条定时单次任务：

要求最近一次的21:30分执行一次当前任务。

任务内容：先站立，等待5秒钟，再趴下。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "124",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "save",</strong><br />
<strong>"mode": "single",</strong><br />
<strong>"condition": "21:30", // 20:47 2022-06-07</strong><br />
<strong>"body": "</strong><br />
<strong>cyberdog.motion.stand_up()</strong><br />
<strong>time.sleep(5)</strong><br />
<strong>cyberdog.motion.get_down()</strong><br />
<strong>"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

保存一条定时周期任务：

要求每天的21:30分执行一次当前任务，直到任务被终止。

任务内容：先站立，等待5秒钟，再趴下。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "125",</strong><br />
<strong>"operate": ["678"],</strong><br />
<strong>"mode": "cycle",</strong><br />
<strong>"condition": "02 15 * * 1,5,7",</strong><br />
<strong>"body": "</strong><br />
<strong>cyberdog.stand_up()</strong><br />
<strong>time.sleep(5)</strong><br />
<strong>cyberdog.get_down()</strong><br />
<strong>"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.1.3 运行任务**

运行一条已经保存的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "run"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.1.4 暂停任务**

暂停一条正在运行的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "suspend"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

暂停多条正在运行的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678","789","891"],</strong><br />
<strong>"operate": "suspend"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.1.5 继续任务**

继续一条已暂停的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "recover"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

继续多条已暂停的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678","789","891"],</strong><br />
<strong>"operate": "recover"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.1.6 终止任务**

终止一条正在运行的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "shutdown"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

终止多条正在运行的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678","789","891"],</strong><br />
<strong>"operate": "shutdown"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.1.7 删除任务**

删除一条已经保存且处于终止状态的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "delete"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

删除多条已经保存且处于终止状态的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678","789","891"],</strong><br />
<strong>"operate": "delete"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.1.8 查询任务**

查询一条已经保存的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "inquiry"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

查询多条已经保存的目标任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678","789","891"],</strong><br />
<strong>"operate": "inquiry"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

查询所有已经保存的任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": [],</strong><br />
<strong>"operate": "inquiry"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.2 module 协议举例**

**0.2.1.1.3.2.1 保存模块**

保存一条带参数的模块。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "module",</strong><br />
<strong>"id": "123",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "save",</strong><br />
<strong>"mode": "common",</strong><br />
<strong>"condition": "usr_behavior_hunger_1(name, size)",</strong><br />
<strong>"body": "</strong><br />
<strong>print("我的", name, "只有 ", size, "%，先趴一会儿。")</strong><br />
<strong>cyberdog.get_down()</strong><br />
<strong>"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

保存一条不带参数的模块。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "module",</strong><br />
<strong>"id": "124",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "save",</strong><br />
<strong>"mode": "common",</strong><br />
<strong>"condition": "usr_behavior_hunger_2()",</strong><br />
<strong>"body": "</strong><br />
<strong>print("我的处于饥饿状态")</strong><br />
<strong>cyberdog.get_down()</strong><br />
<strong>"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

保存一条序列模块。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<br />
{<br />
"type": "module",<br />
"id": "124",<br />
"operate": "add",<br />
"mode": "sequence",<br />
"condition": "usr_behavior_sequence_2()",<br />
"body": "<br />
<br />
sequ = MotionSequence()<br />
sequ.name = 'test_sequ'<br />
sequ.describe = '测试序列'<br />
<br />
# 原始步态配置文件为：./user_gait_00.toml.<br />
<br />
gait_meta = MotionSequenceGait()<br />
<br />
gait_meta.right_forefoot = 1<br />
gait_meta.left_forefoot = 1<br />
gait_meta.right_hindfoot = 1<br />
gait_meta.left_hindfoot = 1<br />
gait_meta.duration = 1<br />
sequ.gait_list.push_back(gait_meta)<br />
<br />
gait_meta.right_forefoot = 0<br />
gait_meta.left_forefoot = 1<br />
gait_meta.right_hindfoot = 1<br />
gait_meta.left_hindfoot = 0<br />
gait_meta.duration = 1<br />
sequ.gait_list.push_back(gait_meta)<br />
<br />
gait_meta.right_forefoot = 1<br />
gait_meta.left_forefoot = 1<br />
gait_meta.right_hindfoot = 1<br />
gait_meta.left_hindfoot = 1<br />
gait_meta.duration = 1<br />
sequ.gait_list.push_back(gait_meta)<br />
<br />
# 原始步伐配置文件为：./L91_user80_ballet_full.toml.<br />
<br />
pace_meta = MotionSequencePace()<br />
<br />
pace_meta.twist.linear.x = 0.000000<br />
pace_meta.twist.linear.y = 0.000000<br />
pace_meta.twist.linear.z = 0.000000<br />
pace_meta.centroid.position.x = 0.000000<br />
pace_meta.centroid.position.y = 0.000000<br />
pace_meta.centroid.position.z = 0.020000<br />
pace_meta.centroid.orientation.x = 0.000000<br />
pace_meta.centroid.orientation.y = 0.000000<br />
pace_meta.centroid.orientation.z = 0.000000<br />
pace_meta.weight.linear.x = 50.000000<br />
pace_meta.weight.linear.y = 50.000000<br />
pace_meta.weight.linear.z = 5.000000<br />
pace_meta.weight.angular.x = 10.000000<br />
pace_meta.weight.angular.y = 10.000000<br />
pace_meta.weight.angular.z = 10.000000<br />
pace_meta.right_forefoot.x = 0.030000<br />
pace_meta.right_forefoot.y = 0.050000<br />
pace_meta.left_forefoot.x = -0.030000<br />
pace_meta.left_forefoot.y = -0.050000<br />
pace_meta.right_hindfoot.x = 0.030000<br />
pace_meta.right_hindfoot.y = 0.050000<br />
pace_meta.left_hindfoot.x = -0.030000<br />
pace_meta.left_hindfoot.y = -0.050000<br />
pace_meta.friction_coefficient = 0.400000<br />
pace_meta.right_forefoot.w = 0.030000<br />
pace_meta.left_forefoot.w = 0.030000<br />
pace_meta.right_hindfoot.w = 0.030000<br />
pace_meta.left_hindfoot.w = 0.030000<br />
pace_meta.landing_gain = 1.500000<br />
pace_meta.use_mpc_track = 0<br />
pace_meta.duration = 300<br />
sequ.pace_list.push_back(pace_meta)<br />
<br />
pace_meta.twist.linear.x = 0.000000<br />
pace_meta.twist.linear.y = 0.000000<br />
pace_meta.twist.linear.z = 0.000000<br />
pace_meta.centroid.position.x = 0.000000<br />
pace_meta.centroid.position.y = 0.000000<br />
pace_meta.centroid.position.z = 0.000000<br />
pace_meta.centroid.orientation.x = 0.000000<br />
pace_meta.centroid.orientation.y = 0.000000<br />
pace_meta.centroid.orientation.z = 0.000000<br />
pace_meta.weight.linear.x = 50.000000<br />
pace_meta.weight.linear.y = 50.000000<br />
pace_meta.weight.linear.z = 5.000000<br />
pace_meta.weight.angular.x = 10.000000<br />
pace_meta.weight.angular.y = 10.000000<br />
pace_meta.weight.angular.z = 10.000000<br />
pace_meta.right_forefoot.x = 0.000000<br />
pace_meta.right_forefoot.y = 0.000000<br />
pace_meta.left_forefoot.x = 0.000000<br />
pace_meta.left_forefoot.y = 0.000000<br />
pace_meta.right_hindfoot.x = 0.000000<br />
pace_meta.right_hindfoot.y = 0.000000<br />
pace_meta.left_hindfoot.x = 0.000000<br />
pace_meta.left_hindfoot.y = 0.000000<br />
pace_meta.friction_coefficient = 0.400000<br />
pace_meta.right_forefoot.w = 0.030000<br />
pace_meta.left_forefoot.w = 0.030000<br />
pace_meta.right_hindfoot.w = 0.030000<br />
pace_meta.left_hindfoot.w = 0.030000<br />
pace_meta.landing_gain = 1.500000<br />
pace_meta.use_mpc_track = 0<br />
pace_meta.duration = 300<br />
sequ.pace_list.push_back(pace_meta)<br />
<br />
cyberdog.motion.run_sequence(sequ)<br />
"<br />
}</td>
</tr>
</tbody>
</table>

调用刚才添加的模块。

<table>
<tbody>
<tr class="odd">
<td>Python<br />
<strong>usr_behavior_hunger_1('电量', 16)</strong><br />
<strong>usr_behavior_hunger_2()</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.2.2 删除模块**

删除一条已经保存且处于终止状态的目标模块。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "module",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "delete"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

删除多条已经保存且处于终止状态的目标模块。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "module",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678","789","891"],</strong><br />
<strong>"operate": "delete"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.2.3 查询模块**

查询一条已经保存的目标模块。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "module",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "inquiry"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

查询多条已经保存的目标。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "module",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678","789","891"],</strong><br />
<strong>"operate": "inquiry"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

查询所有已经保存的模块。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "module",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": [],</strong><br />
<strong>"operate": "inquiry"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.3 AI 协议举例**

**0.2.1.1.3.3.1 查询底库人员信息**

查询一条已经保存的底库人员信息。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "AI",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "inquiry",</strong><br />
<strong>"mode": "personnel",</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

查询多条已经保存的底库人员信息。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "AI",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": ["678","789","891"],</strong><br />
<strong>"operate": "inquiry",</strong><br />
<strong>"mode": "personnel",</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

查询所有已经保存的任务。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "AI",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": [],</strong><br />
<strong>"operate": "inquiry",</strong><br />
<strong>"mode": "personnel",</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.1.3.3.2 查询底库人脸信息**

查询底库所有已经录入人脸的人员。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "AI",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": [],</strong><br />
<strong>"operate": "inquiry",</strong><br />
<strong>"mode": "face",</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

其他查询类举。

**0.2.1.1.3.3.3 查询底库声纹信息**

查询底库所有已经录入声纹的人员。

<table>
<tbody>
<tr class="odd">
<td><blockquote>
<p>JSON<br />
<strong>{</strong><br />
<strong>"type": "AI",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": [],</strong><br />
<strong>"operate": "inquiry",</strong><br />
<strong>"mode": "voiceprint",</strong><br />
<strong>}</strong></p>
</blockquote></td>
</tr>
</tbody>
</table>

其他查询类举。

**0.2.1.1.3.4 SLAM 协议举例**

**0.2.1.1.3.4.1 查询当前地图中的预置点信息**

查询当前地图中的预置点。

<table>
<tbody>
<tr class="odd">
<td><blockquote>
<p>JSON<br />
<strong>{</strong><br />
<strong>"type": "SLAM",</strong><br />
<strong>"id": "xxx",</strong><br />
<strong>"target_id": [],</strong><br />
<strong>"operate": "inquiry",</strong><br />
<strong>"mode": "preset",</strong><br />
<strong>}</strong></p>
</blockquote></td>
</tr>
</tbody>
</table>

**0.2.1.2 \[机器人端-\> 前端\]/ \[机器人端-\> 后端\]：上报数据json格式**

**0.2.1.2.1 协议约束**

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"feedback": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"target_id": "target_id",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"state": 0,</strong><br />
<strong>"describe": "describe"</strong><br />
<strong>},</strong><br />
<strong>"block": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id"</strong><br />
<strong>},</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"list": [</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"dependent": ["","",...],</strong><br />
<strong>"be_depended": ["","",...],</strong><br />
<strong>"describe": "describe"</strong><br />
<strong>},</strong><br />
...<br />
<strong>]</strong><br />
<strong>},</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.2.2 协议说明**

**0.2.1.2.2.1 普通请求反馈**

<table>
<tbody>
<tr class="odd">
<td>Python<br />
<strong>{</strong><br />
<strong>"feedback": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"target_id": "target_id",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"state": 0,</strong><br />
<strong>"describe": "describe"</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**feedback:** \[必须\]请求消息反馈

**feedback.type**：\[必须\]消息类型

合法参数：

任务:'task'

模块:'module'

**feedback.id**：\[必须\] 消息id

响应请求消息时，为原始消息id；

反馈任务状态时，为时间戳。

**feedback.target\_id**：\[必须\] 消息target\_id

响应请求消息时，如果原始消息target\_id非空，则为首个元素，反之为空；

反馈任务状态时，为任务target\_id。

**feedback.operate**：\[必须\] 消息操作

app触发的操作：

调试任务/模块:'debug'

添加任务/模块:'save'

修改任务/模块:'run'

删除任务/模块:'delete'

查询任务/模块:'inquiry'

终止任务:'shutdown'

暂停任务:'suspend'

继续任务:'recover'

任务自动触发的操作：

开始任务:'start'

停止任务:'stop'

机器人想服务端反馈的操作：

开始任务:'start'

**feedback.state**：\[必须\] 原始消息操作状态

值约束：

0:当前操作成功

非0:切换为当前指定类型发生异常

1: 非json格式

2: type字段，类型无效

3: id字段，id无效

4: target\_id字段，id无效

5: describe字段，描述无效

6: style字段，样式无效

7: operate字段，操作无效

8: mode字段，模式无效

9: condition字段，条件无效

10: body字段，主体无效

21: 无法创建路径

22: 无法打开文件

23: body字段语法异常，无法构建

24: 无法注册

25: 无法更新列表

26: 无法执行

27: 当前操作非法

28: 请求错误

29: 其他错误

30: 服务被打断

31: 等待服务上线超时

31: 请求服务超时

**feedback.describe**：\[必须\] 原始消息操作状态描述

**注意**：当请求操作为下述几个时，会收到两次反馈。

终止任务:'shutdown'

暂停任务:'suspend'

继续任务:'recover'

其中：两次反馈的主要区别是：**describe**字段：

第一次是引擎反馈收到操作请求且请求合法；

第二次是任务响应请求**成功后**的反馈，**describe**字段格式为：“Task loop feedback, now state is xxx”。

**0.2.1.2.2.2 任务执行过程中反馈**

<table>
<tbody>
<tr class="odd">
<td>Python<br />
<strong>{</strong><br />
<strong>"feedback": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"target_id": "target_id",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"state": 0,</strong><br />
<strong>"describe": "describe"</strong><br />
<strong>},</strong><br />
<strong>"block": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id"</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**feedback:** \[必须\]请求消息反馈

**feedback.type**：\[必须\]消息类型

合法参数：

任务:'task'

模块:'module'

**feedback.id**：\[必须\] 消息id

响应请求消息时，为原始消息id；

反馈任务状态时，为时间戳。

**feedback.target\_id**：\[必须\] 消息target\_id

响应请求消息时，如果原始消息target\_id非空，则为首个元素，反之为空；

反馈任务状态时，为任务target\_id。

**feedback.operate**：\[必须\] 消息操作

app触发的操作：

调试任务/模块:'debug'

添加任务/模块:'save'

修改任务/模块:'run'

删除任务/模块:'delete'

查询任务/模块:'inquiry'

终止任务:'shutdown'

暂停任务:'suspend'

继续任务:'recover'

任务自动触发的操作：

开始任务:'start'

停止任务:'stop'

机器人想服务端反馈的操作：

开始任务:'start'

**feedback.state**：\[必须\] 原始消息操作状态

值约束：

0:当前操作成功

非0:切换为当前指定类型发生异常

1: 非json格式

2: type字段，类型无效

3: id字段，id无效

4: target\_id字段，id无效

5: describe字段，描述无效

6: style字段，样式无效

7: operate字段，操作无效

8: mode字段，模式无效

9: condition字段，条件无效

10: body字段，主体无效

21: 无法创建路径

22: 无法打开文件

23: body字段语法异常，无法构建

24: 无法注册

25: 无法更新列表

26: 无法执行

27: 当前操作非法

28: 请求错误

29: 其他错误

30: 服务被打断

31: 等待服务上线超时

31: 请求服务超时

**feedback.describe**：\[必须\] 原始消息操作状态描述

**block:** \[可选\]块信息

**block.type**：\[必须\] 消息类型

合法参数：

开始执行目标块:'begin'

结束执行目标块:'end'

**block**.**id**：\[必须\] 块id

**0.2.1.2.2.3 任务、模块查询反馈**

<table>
<tbody>
<tr class="odd">
<td>Python<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"list": [</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style"，</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition"，</strong><br />
<strong>"dependent": [],</strong><br />
<strong>"be_depended": []</strong><br />
<strong>},</strong><br />
...<br />
<strong>]</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**response:** \[可选\]用于查询请求的回复信息格式约束

**response.type**：\[必须\] 消息类型

合法参数：

表示当前列表类型为任务列表:'**task**'

表示当前列表类型为模块列表:'**module**'

**response.id**：\[必须\] 消息id

合法参数：

保持和请求id一致。

**response**.**list**：\[必须\] 响应列表

**response.list\[\].id：**\[必须\] 列表元素的id

根据查询场景可分为：

任务id；

模块id；

数据和下发数据帧该字段保持一致

**response.list\[\].describe：**\[必须\] 列表元素的描述

任务描述 或 模块描述

数据和下发数据帧保持一致

**response.list\[\].style**: \[可选\] 字符串类型，标识当前消息样式，即当前执行主体内容的呈现方式。

任务样式 或 模块样式

数据和下发数据帧该字段保持一致

**response.list\[\].operate：**\[必须\] 列表元素的状态

任务状态 或 模块状态，合法值约束如下：

"null" ： \[状态\]无状态（仅保存的内容）

"error"： \[状态\]错误状态

"wait\_run" ： \[状态\]等待运行状态

"run\_wait" ： \[状态\]运行等待状态

"run" ： \[状态\]运行状态

"suspend" ： \[状态\]暂停

"shutdown" ：\[状态\]终止

模块状态，合法值约束如下：

"null" ： \[状态\]无状态（仅保存的内容）

"error"： \[状态\]错误状态

"normal" ： \[状态\]正常状态

**response.list\[\].mode：**\[必须\] 列表元素的模式类型

任务模式类型 或 模块模式类型

数据和下发数据帧该字段保持一致

**response.list\[\].condition：**\[必须\] 列表元素的约束条件

任务执行约束 或 模块接口约束

数据和下发数据帧该字段保持一致

**response.list\[\]. dependent\[\]：**\[必须\] 列表元素的依赖模块，用于标识当前任务或当前模块依赖哪些模块，也就是调用了哪些模块；

**response.list\[\].be\_depended\[\]：**\[必须\] 列表元素（只有模块才会被依赖）的被依赖项，被依赖的可能是任务也可能是模块。

**注意：**当被依赖集合非空（存在其他任务或模块依赖当前模块）时，不允许删除模块。

**0.2.1.2.2.4 人员信息查询反馈**

<table>
<tbody>
<tr class="odd">
<td>Python<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"list": [</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>},</strong><br />
...<br />
<strong>]</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**response:** \[可选\]用于查询请求的回复信息格式约束

**response.type**：\[必须\] 消息类型

合法参数：

表示当前列表类型为人工智能列表:'AI'

**response.id**：\[必须\] 消息id

合法参数：

保持和请求id一致。

**response**.**list**：\[必须\] 响应列表

**response.list\[\].id：**\[必须\] 列表元素的id

根据查询场景可分为：

人员id;

数据和下发数据帧该字段保持一致

**response.list\[\].mode：**\[必须\] 列表元素的模式类型

人员类型 或 人脸类型 或 声纹类型

数据和下发数据帧该字段保持一致

**response.list\[\].style:** \[必须\] 字符串类型，标识当前人员人脸录入状态。合法值如下：

"0"：未录入；

"1"：已录入；

"2"：正在录入；

**response.list\[\].condition:** \[必须\] 字符串类型，标识当前人员声纹录入状态。合法值如下：

"0"：未录入；

"1"：已录入；

"2"：正在录入；

**response.list\[\].describe：**\[必须\] 列表元素的描述

人员描述

**0.2.1.2.2.5 训练词查询反馈**

<table>
<tbody>
<tr class="odd">
<td>Python<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"list": [</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"describe": "describe"，</strong><br />
<strong>},</strong><br />
...<br />
<strong>]</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**response:** \[可选\]用于查询请求的回复信息格式约束

**response.type**：\[必须\] 消息类型

合法参数：

表示当前列表类型为人工智能列表:'AI'

**response.id**：\[必须\] 消息id

合法参数：

保持和请求id一致。

**response**.**list**：\[必须\] 响应列表

**response.list\[\].id：**\[必须\] 列表元素的id

根据查询场景为训练词id，用于呈现给用户（trigger）

**response.list\[\].mode：**\[必须\] 列表元素的模式类型

训练词类型

**response.list\[\].style：**\[必须\] 字符串类型，标识样式（type）

**response.list\[\].condition：**\[必须\] 字符串类型，标识训练词约束信息（value）

**response.list\[\].describe：**\[必须\] 列表元素的描述

训练词描述

**0.2.1.2.2.6 AI所有信息查询反馈**

<table>
<tbody>
<tr class="odd">
<td>Python<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"list": [</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"describe": "describe"，</strong><br />
<strong>},</strong><br />
...<br />
<strong>]</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**response:** \[可选\]用于查询请求的回复信息格式约束

**response.type**：\[必须\] 消息类型

合法参数：

表示当前列表类型为人工智能列表:'AI'

**response.id**：\[必须\] 消息id

合法参数：

保持和请求id一致。

**response**.**list**：\[必须\] 响应列表

**response.list\[\].id：**\[必须\] 列表元素的id

根据查询场景为训练词id，用于呈现给用户（trigger）

**response.list\[\].mode：**\[必须\] 列表元素的模式类型

personnel：人员类型

training\_words：训练词类型

**response.list\[\].style：**\[必须\] 列表元素的样式（类型）

当mode为personnel时：标识当前人员人脸录入状态。合法值如下：

"0"：未录入；

"1"：已录入；

"2"：正在录入；

当mode为training\_words时：标识当前训练词样式（type）

**response.list\[\]. condition：**\[必须\] 列表元素的约束

当mode为personnel时：标识当前人员声纹录入状态。合法值如下：

"0"：未录入；

"1"：已录入；

"2"：正在录入；

当mode为training\_words时：标识当前训练词操作（value）

**response.list\[\].describe：**\[必须\] 列表元素的描述

训练词描述

**0.2.1.2.2.7 地图预置点查询反馈**

<table>
<tbody>
<tr class="odd">
<td>Python<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"list": [</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"mode": "mode"</strong><br />
<strong>"style": "style"，</strong><br />
<strong>"condition": "condition"</strong><br />
<strong>"describe": "describe"，</strong><br />
<strong>},</strong><br />
...<br />
<strong>]</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**response:** \[可选\]用于查询请求的回复信息格式约束

**response.type**：\[必须\] 消息类型

合法参数：

表示当前列表类型为人工智能列表:'SLAM'

**response.id**：\[必须\] 消息id

合法参数：

保持和请求id一致。

**response**.**list**：\[必须\] 响应列表

**response.list\[\].id：**\[必须\] 列表元素的id

根据查询场景可分为：

预置点id;

数据和下发数据帧该字段保持一致

**response.list\[\].describe：**\[必须\] 列表元素的描述

预置点描述

**response.list\[\].mode：**\[必须\] 列表元素的模式类型

预置点类型

数据和下发数据帧该字段保持一致

**response.list\[\].style：**\[必须\] 列表元素的样式类型

存放预置点坐标，数据格式如下：

“\[x, y, z\]”

**response.list\[\].condition：**\[必须\] 列表元素的约束类型

预置点所在地图名

**0.2.1.2.2.1 协议说明**

**0.2.1.2.3 协议举例**

下发一条保存定时单次任务：

要求每天的21:30分执行一次当前任务，但第一次执行后就销毁，不再执行。

任务内容：先站立，等待5秒钟，再趴下。

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "123",</strong><br />
<strong>"target_id": ["678"],</strong><br />
<strong>"operate": "save",</strong><br />
<strong>"mode": "single",</strong><br />
<strong>"condition": "now",</strong><br />
<strong>"body": "</strong><br />
<strong>cyberdog.block('block_01')</strong><br />
<strong>cyberdog.stand_up()</strong><br />
<strong>cyberdog.block('block_02')</strong><br />
<strong>time.sleep(60)</strong><br />
<strong>cyberdog.block('block_03')</strong><br />
<strong>cyberdog.get_down()</strong><br />
<strong>"</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.2.3.1 上报任务过程**

保存成功时上报（以成功场景为例）：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"feedback": {</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "0123",</strong><br />
<strong>"target_id": "678",</strong><br />
<strong>"operate": "save",</strong><br />
<strong>"state": 0,</strong><br />
<strong>"describe": "save task"</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

开始执行任务时上报（以成功场景为例）：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"feedback": {</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "1123",</strong><br />
<strong>"target_id": "678",</strong><br />
<strong>"operate": "start",</strong><br />
<strong>"state": 0,</strong><br />
<strong>"describe": "start task"</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.2.3.2 开始执行当前任务块1时上报**

开始执行当前任务块1时上报：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"feedback": {</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "2123",</strong><br />
<strong>"target_id": "678",</strong><br />
<strong>"operate": "run",</strong><br />
<strong>"state": 0,</strong><br />
<strong>"describe": "run task"</strong><br />
<strong>},</strong><br />
<strong>"block": {</strong><br />
<strong>"type": "begin",</strong><br />
<strong>"id": "block_01"</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

其他块以此类推。

**0.2.1.2.3.3 结束执行当前任务块1时上报**

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"feedback": {</strong><br />
<strong>"type": "task",</strong><br />
<strong>"id": "5123",</strong><br />
<strong>"target_id": "678",</strong><br />
<strong>"operate": "run",</strong><br />
<strong>"state": 0,</strong><br />
<strong>"describe": "run task"</strong><br />
<strong>},</strong><br />
<strong>"block": {</strong><br />
<strong>"type": "end",</strong><br />
<strong>"id": "block_01"</strong><br />
<strong>}</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

其他块以此类推。

**0.2.1.2.3.4 查询任务列表**

当模块列表为空时返回值：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "task",</strong><br />
<strong>"list": []</strong><br />
<strong>},</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

当模块列表非空时返回值：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "module",</strong><br />
<strong>"list": [</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style"，</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition"</strong><br />
<strong>},</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style"，</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition"</strong><br />
<strong>},</strong><br />
<strong>...</strong><br />
<strong>]</strong><br />
<strong>},</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.2.3.5 查询模块列表**

当模块列表为空时返回值：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "module",</strong><br />
<strong>"list": []</strong><br />
<strong>},</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

当模块列表非空时返回值：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"response": {</strong><br />
<strong>"type": "task",</strong><br />
<strong>"list": [</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style"，</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition"</strong><br />
<strong>},</strong><br />
<strong>{</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style"，</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition"</strong><br />
<strong>},</strong><br />
<strong>...</strong><br />
<strong>]</strong><br />
<strong>},</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.3 后端-\>机器人端：上报数据json格式**

**0.2.1.3.1 协议约束**

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"code": "code",</strong><br />
<strong>"message": "message",</strong><br />
<strong>"request_id": "request_id",</strong><br />
<strong>"data": [</strong><br />
<strong>{</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"body": "body"</strong><br />
<strong>},</strong><br />
<strong>{</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"body": "body"</strong><br />
<strong>}</strong><br />
...<br />
<strong>]</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.3.2 协议说明**

**code**：\[必须\] 字符串类型，标识当前帧状态码；

200 成功

400 访问超时

403 拒绝访问

401 未经授权访问

404 资源不存在

405 不支持当前请求方法

500 服务器运行异常

10001 参数不能为空

**message**：\[必须\] 字符串类型，标识当前帧状态码描述；

**request\_id**：\[必须\] 字符串类型，标识当前帧id；

**data:** \[可选\]块信息，当响应查询请求时是增量查询，即只返回未同步到机器人的收藏内容。

**data.type**：\[必须\] 消息类型

合法参数：

表示当前列表类型为任务列表:'**task**'

表示当前列表类型为模块列表:'**module**'

表示当前列表类型为记录列表:'**record**'

**data**.**list**：\[必须\] 响应列表

**data.list\[\].id：**\[必须\] 列表元素的id

任务id 或 模块id

数据和下发数据帧该字段保持一致

**data.list\[\].describe：**\[必须\] 列表元素的描述

任务描述 或 模块描述

数据和下发数据帧保持一致

**data.list\[\].style**: \[可选\] 字符串类型，标识当前消息样式，即当前执行主体内容的呈现方式。

任务样式 或 模块样式

数据和下发数据帧该字段保持一致

**data.list\[\].operate：**\[必须\] 列表元素的状态

任务状态 或 模块状态

数据为当前真实状态

**data.list\[\].mode：**\[必须\] 列表元素的模式类型

任务模式类型 或 模块模式类型

数据和下发数据帧该字段保持一致

**data.list\[\].condition：**\[必须\] 列表元素的约束条件

任务执行约束 或 模块接口约束

数据和下发数据帧该字段保持一致

**data.list\[\].body：**\[必须\] 列表元素的脚本数据

任务脚本 或 模块脚本

数据和下发数据帧该字段保持一致

**data.list\[\].state：**\[必须\] 列表元素的脚本状态，合法值如下：

任务状态 或 模块状态，合法值约束如下：

"null" ： \[状态\]无状态（仅保存的内容）

"error"： \[状态\]错误状态

"wait\_run" ： \[状态\]等待运行状态

"run\_wait" ： \[状态\]运行等待状态

"run" ： \[状态\]运行状态

"suspend" ： \[状态\]暂停

"shutdown" ：\[状态\]终止

模块状态，合法值约束如下：

"null" ： \[状态\]无状态（仅保存的内容）

"error"： \[状态\]错误状态

"normal" ： \[状态\]正常状态

**0.2.1.3.3 协议举例**

**0.2.1.3.3.1 查询任务列表**

当模块列表为空时返回值：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"code": "code",</strong><br />
<strong>"message": "message",</strong><br />
<strong>"request_id": "request_id",</strong><br />
<strong>"data": []</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

当模块列表非空时返回值：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"code": "code",</strong><br />
<strong>"message": "message",</strong><br />
<strong>"request_id": "request_id",</strong><br />
<strong>"data": [</strong><br />
<strong>{</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"body": "body"</strong><br />
<strong>},</strong><br />
<strong>{</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"body": "body"</strong><br />
<strong>}</strong><br />
<strong>...</strong><br />
<strong>]</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.1.3.3.2 查询模块列表**

当模块列表为空时返回值：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"code": "code",</strong><br />
<strong>"message": "message",</strong><br />
<strong>"request_id": "request_id",</strong><br />
<strong>"data": []</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

当模块列表非空时返回值：

<table>
<tbody>
<tr class="odd">
<td>JSON<br />
<strong>{</strong><br />
<strong>"code": "code",</strong><br />
<strong>"message": "message",</strong><br />
<strong>"request_id": "request_id",</strong><br />
<strong>"data": [</strong><br />
<strong>{</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"body": "body"</strong><br />
<strong>},</strong><br />
<strong>{</strong><br />
<strong>"type": "type",</strong><br />
<strong>"id": "id",</strong><br />
<strong>"describe": "describe",</strong><br />
<strong>"style": "style",</strong><br />
<strong>"operate": "operate",</strong><br />
<strong>"mode": "mode",</strong><br />
<strong>"condition": "condition",</strong><br />
<strong>"body": "body"</strong><br />
<strong>}</strong><br />
<strong>...</strong><br />
<strong>]</strong><br />
<strong>}</strong></td>
</tr>
</tbody>
</table>

**0.2.2 机器人端状态流转关系**

**0.2.2.1 机器人端任务状态流转关系**

|                 |             |             |            |             |         |             |             |              |           |             |
| --------------- | ----------- | ----------- | ---------- | ----------- | ------- | ----------- | ----------- | ------------ | --------- | ----------- |
| 状态流转关系表         |             |             |            |             |         |             |             |              |           |             |
| 当前状态            | 当前操作请求      |             |            |             |         |             |             |              | 任务自动触发    |             |
|                 | 查询(inquiry) | 保存(save)    | 删除(delete) | 调试(debug)   | 运行(run) | 暂停(suspend) | 继续(recover) | 终止(shutdown) | 启动(start) | 停止(stop)    |
| 空（null）         | 空状态         | 错误状态,等待运行状态 | 空状态        | 错误状态,运行等待状态 | 非法操作    | 非法操作        | 非法操作        | 非法操作         | 非法操作      | 非法操作        |
| 错误(error)       | 错误状态        | 错误状态,等待运行状态 | 空状态        | 错误状态,运行等待状态 | 非法操作    | 非法操作        | 非法操作        | 非法操作         | 非法操作      | 非法操作        |
| 等待运行(wait\_run) | 等待运行状态      | 错误状态,等待运行状态 | 空状态        | 错误状态,运行等待状态 | 运行等待状态  | 非法操作        | 非法操作        | 终止状态         | 非法操作      | 非法操作        |
| 运行等待(run\_wait) | 运行等待状态      | 非法操作        | 非法操作       | 错误状态,运行等待状态 | 非法操作    | 非法操作        | 非法操作        | 终止状态         | 运行状态      | 非法操作        |
| 运行(run)         | 运行状态        | 非法操作        | 非法操作       | 错误状态,运行等待状态 | 运行状态    | 暂停状态        | 运行状态        | 运行等待状态,终止状态  | 运行状态      | 运行等待状态,终止状态 |
| 暂停(suspend)     | 暂停状态        | 非法操作        | 非法操作       | 错误状态,运行等待状态 | 非法操作    | 暂停状态        | 运行状态        | 运行等待状态,终止状态  | 非法操作      | 运行等待状态,终止状态 |
| 终止(shutdown)    | 终止状态        | 错误状态,等待运行状态 | 空状态        | 错误状态,运行等待状态 | 运行等待状态  | 非法操作        | 非法操作        | 终止状态         | 非法操作      | 非法操作        |

如上表所示，七种任务状态分别在八种任务操作下的流转关系尽收眼底，现对八种任务操作及七种任务状态进行说明如下：

**八种任务操作**

保存任务：构建当前id的任务，若任务id已存在则覆盖，并审核任务语法是否合规，反馈操作结果。

运行任务：运行当前id对应的任务，若任务id对应的任务不存在或语法状态错误则不执行，反馈操作结果。

查询任务：查询当前id对应的任务，反馈操作结果。

删除任务：删除当前id对应的任务，反馈操作结果。

暂停任务：暂停当前id对应的任务，反馈操作结果。

继续任务：继续当前id对应的任务，反馈操作结果。

终止任务：终止当前id对应的任务，反馈操作结果。

调试任务：以当前调试id为基础，保存、审核及运行当前帧携带的逻辑，反馈操作结果。

**七种任务状态**

空状态：该状态是指没有当前任务的任何状态，本质为当前任务不存在，也就是说任何未记录的任务的状态均为空状态。

错误状态：该状态是指当前任务不符合语法规则，本质为当前任务不合规，也就是说当前任务无法运行只能再次编辑。

等待运行状态：该状态是指当前任务可以运行但尚未加入任务注册表中，本质为当前任务语法规则，也就是说当前任务处于等待用户确认运行状态。

运行等待状态：该状态是指当前任务已加入任务注册表中，但尚未满足运行条件，本质为当前任务语法规则，且正在等待运行条件满足后即刻运行的状态。

运行状态：该状态是指当前任务已满足执行条件，正在执行内部逻辑的状态。

暂停状态：该状态是指当前任务处于暂停执行的状态，此时任务进程任在，可以是断点暂停或用户手动暂停正在执行的任务。

终止状态：该状态是指当前任务被终止执行，此时不存在任务进程，可以是正常执行结束或被迫终止任务。

**注意**：当前的任务状态是基于任务内编程逻辑考虑，不做任务执行约束条件考虑（由调用方考虑）：

当某条任务内编程逻辑审核通过，该任务状态即为等待运行（wait\_run）状态，若该任务执行约束条件为定时单次执行时，需要调用方根据定时约束判断当前任务的约束时间是否已过期，当未过期时再开启任务。当然，也可以不用管，直接下发开启请求，但是会被视为非法请求，操作会失败，对用户操作体验不友好。

**0.2.2.2 机器人端模块状态流转关系**

|            |             |           |            |
| ---------- | ----------- | --------- | ---------- |
| 状态流转关系表    |             |           |            |
| 当前状态       | 当前操作请求      |           |            |
|            | 查询(inquiry) | 保存(save)  | 删除(delete) |
| 空（null）    | 空状态         | 错误状态,正常状态 | 非法操作       |
| 错误(error)  | 错误状态        | 错误状态,正常状态 | 空状态        |
| 正常(normal) | 正常状态        | 错误状态,正常状态 | 空状态,正常状态   |

如上表所示，三种模块状态分别在三种模块操作下的流转关系尽收眼底，现对三种模块操作及三种模块状态进行说明如下：

**三种模块操作**

保存任务：构建当前id的模块，若模块id已存在则覆盖，并审核任务语法是否合规，反馈操作结果。

查询任务：查询当前id对应的模块，反馈操作结果。

删除任务：审核当前id对应的模块是否可以删除，若可以则删除当前id对应的模块，反馈操作结果。

**三种模块状态**

空状态：该状态是指没有当前模块的任何状态，本质为当前模块不存在，也就是说任何未记录的模块的状态均为空状态。

错误状态：该状态是指当前模块不符合语法规则，也就是说当前模块无法运行只能再次编辑。

正常状态：该状态是指当前模块符合语法规则，也就是说当前模块可以调用。

**0.2.2.3 机器人端各模式下操作约束**

> 非法操作会被拒绝，合法操作会被采纳，不管是非法还是合法都会向请求方反馈状态。

|              |               |                           |             |            |           |         |             |             |              |           |          |
| ------------ | ------------- | ------------------------- | ----------- | ---------- | --------- | ------- | ----------- | ----------- | ------------ | --------- | -------- |
| 机器人各模式下操作约束表 |               |                           |             |            |           |         |             |             |              |           |          |
| 当前模式         |               | 用户对任务、模块、AI能力及SLAM能力的查询操作 | 用户对任务或模块的操作 |            | 用户对任务的操作  |         |             |             |              | 任务自动触发    |          |
| 模式名称         | 模式标识          | 查询(inquiry)               | 保存(save)    | 删除(delete) | 调试(debug) | 运行(run) | 暂停(suspend) | 继续(recover) | 终止(shutdown) | 启动(start) | 停止(stop) |
| 未初始化         | Uninitialized | 非法操作                      | 非法操作        | 非法操作       | 非法操作      | 非法操作    | 非法操作        | 非法操作        | 非法操作         | 非法操作      | 非法操作     |
| 资源加载模式       | SetUp         | 非法操作                      | 非法操作        | 非法操作       | 非法操作      | 非法操作    | 非法操作        | 非法操作        | 非法操作         | 非法操作      | 非法操作     |
| 资源释放模式       | TearDown      | 非法操作                      | 非法操作        | 非法操作       | 非法操作      | 非法操作    | 非法操作        | 非法操作        | 非法操作         | 非法操作      | 非法操作     |
| 自检模式         | SekfCheck     | 合法操作                      | 非法操作        | 非法操作       | 非法操作      | 非法操作    | 合法操作        | 非法操作        | 合法操作         | 非法操作      | 合法操作     |
| 活跃模式         | Active        | 合法操作                      | 合法操作        | 合法操作       | 合法操作      | 合法操作    | 合法操作        | 合法操作        | 合法操作         | 合法操作      | 合法操作     |
| 静默模式         | DeActive      | 合法操作                      | 合法操作        | 合法操作       | 非法操作      | 非法操作    | 合法操作        | 非法操作        | 合法操作         | 非法操作      | 合法操作     |
| 低电量模式        | Protected     | 合法操作                      | 合法操作        | 合法操作       | 合法操作      | 合法操作    | 合法操作        | 合法操作        | 合法操作         | 合法操作      | 合法操作     |
| 低功耗模式        | LowPower      | 合法操作                      | 合法操作        | 合法操作       | 非法操作      | 非法操作    | 合法操作        | 非法操作        | 合法操作         | 非法操作      | 合法操作     |
| 远程升级模式       | OTA           | 合法操作                      | 非法操作        | 非法操作       | 非法操作      | 非法操作    | 合法操作        | 非法操作        | 合法操作         | 非法操作      | 合法操作     |
| 错误模式         | Error         | 合法操作                      | 非法操作        | 非法操作       | 非法操作      | 非法操作    | 合法操作        | 非法操作        | 合法操作         | 非法操作      | 合法操作     |

如上表所示，十种机器人模式下用户对任务或模块的八种操作，用户对AI能力的一种操作以及任务自行触发的两种操作约束条件尽收眼底，各模式下的约束指标主要考虑如下：

Uninitialized、SetUp、TearDown三种模式下，机器人各功能模块均处于无法正常工作的状态，故而限制所有操作；

SekfCheck、OTA以及Error模式下，机器人各功能模块均处于封闭状态，故而限制所有可能导致图形化编程新增进程的操作，只允许现有进程自动停止（stop）或用户手动暂停（suspend）或终止（shutdown）现有进程，允许查询（inquiry）是想为用户提供展示任务、模块及AI信息，为用户的暂停（suspend）或终止提供基础，同时也支持二次编辑任务或模块的功能，提升用户和机器人互动过程中的异步体验（不会因自检阻塞用户编程），该思想同样适用于Active、DeActive、Protected、LowPower、OTA以及Error模式；

Active模式下，机器人各功能模块均处于能够正常工作的阶段，故而开放所有操作；

Protected模式下，机器人大部分功能模块均处于能够正常工作的阶段，故而开放所有操作，该模式下受限功能如下：

运动模块：除站立、趴下外的所有结果指令；

LED模块：BMS会抢占LED设备。

DeActive以及LowPower模式下，机器人各功能模块均处于休眠状态，查询（inquiry）、停止（stop）及暂停（suspend）或终止（shutdown）约束考虑和SekfCheck模式一样，对于保存（save）和删除（delete）操作的考虑和查询一样，也是想为用户提供展示任务、模块及AI信息，为用户的暂停（suspend）或终止提供基础，同时也支持二次编辑任务或模块的功能，提升用户和机器人互动过程中的异步体验（不会因自检阻塞用户编程）；

<table>
<tbody>
<tr class="odd">
<td><p>低电量模式</p>
<p>进入：</p>
<p>电量低于20%，自动进入；</p>
<p>退出：</p>
<p>电量大于等于20%，自动退出；</p>
<p>低功耗模式</p>
<p>进入：</p>
<p>电量小于5%时自动进入低功耗；</p>
<p>趴下超过30s，进入低功耗；</p>
<p>唤醒退出低功耗模式后，若30s内未进行运动控制，则再次进入低功耗模式；</p>
<p>退出：</p>
<p>语音唤醒：“铁蛋、铁蛋”；</p>
<p>app端点击退出低功耗；</p>
<p>双击狗头退出低功耗；</p></td>
</tr>
</tbody>
</table>

**一、基础约束**

**1.1 保留字**

|        |         |      |          |        |        |      |        |       |        |
| ------ | ------- | ---- | -------- | ------ | ------ | ---- | ------ | ----- | ------ |
| and    | exec    | not  | class    | from   | print  | del  | import | try   | except |
| assert | finally | or   | continue | global | raise  | elif | in     | while | lambda |
| break  | for     | pass | def      | if     | return | else | is     | with  | yield  |

**1.2 运算符**

**1.2.1 算术运算符**

<table>
<tbody>
<tr class="odd">
<td><strong>运算符</strong></td>
<td><strong>描述</strong></td>
<td><strong>实例（a=10，b=20）</strong></td>
</tr>
<tr class="even">
<td><strong>+</strong></td>
<td>加：两个对象相加</td>
<td>a + b 输出结果 30</td>
</tr>
<tr class="odd">
<td><strong>-</strong></td>
<td>减：得到负数或是一个数减去另一个数</td>
<td>a - b 输出结果 -10</td>
</tr>
<tr class="even">
<td><strong>*</strong></td>
<td>乘：两个数相乘或是返回一个被重复若干次的字符串</td>
<td>a * b 输出结果 200</td>
</tr>
<tr class="odd">
<td><strong>/</strong></td>
<td>除：x除以y</td>
<td>b / a 输出结果 2</td>
</tr>
<tr class="even">
<td><strong>%</strong></td>
<td>取模：返回除法的余数</td>
<td>b % a 输出结果 0</td>
</tr>
<tr class="odd">
<td><strong>**</strong></td>
<td>幂：返回x的y次幂</td>
<td>a**b 为10的20次方， 输出结果 100000000000000000000</td>
</tr>
<tr class="even">
<td><strong>//</strong></td>
<td>取整除：返回商的整数部分（向下取整）</td>
<td><p>9//2 输出结果 4</p>
<p>-9//2 输出结果 -5</p></td>
</tr>
</tbody>
</table>

**1.2.2 比较运算符**

所有比较运算符返回1表示真，返回0表示假。这分别与特殊的变量True和False等价。

|         |                  |                     |
| ------- | ---------------- | ------------------- |
| **运算符** | **描述**           | **实例（a=10，b=20）**   |
| **==**  | 等于： 比较对象是否相等     | (a == b) 返回 false。  |
| **\!=** | 不等于：比较两个对象是否不相等  | (a \!= b) 返回 true.  |
| **\>**  | 大于：返回x是否大于y      | (a \> b) 返回 false。  |
| **\<**  | 小于：返回x是否小于y。     | (a \< b) 返回 true。   |
| **\>=** | 大于等于：返回x是否大于等于y。 | (a \>= b) 返回 false。 |
| **\<=** | 小于等于：返回x是否小于等于y。 | (a \<= b) 返回 true。  |

**1.2.3 赋值运算符**

|           |          |                              |
| --------- | -------- | ---------------------------- |
| **运算符**   | **描述**   | **实例（a=10，b=20）**            |
| **=**     | 简单的赋值运算符 | c = a + b 将 a + b 的运算结果赋值为 c |
| **+=**    | 加法赋值运算符  | c += a 等效于 c = c + a         |
| **-=**    | 减法赋值运算符  | c -= a 等效于 c = c - a         |
| **\*=**   | 乘法赋值运算符  | c \*= a 等效于 c = c \* a       |
| **/=**    | 除法赋值运算符  | c /= a 等效于 c = c / a         |
| **%=**    | 取模赋值运算符  | c %= a 等效于 c = c % a         |
| **\*\*=** | 幂赋值运算符   | c \*\*= a 等效于 c = c \*\* a   |
| **//=**   | 取整除赋值运算符 | c //= a 等效于 c = c // a       |

**1.2.4 逻辑运算符**

|         |           |                                                      |                       |
| ------- | --------- | ---------------------------------------------------- | --------------------- |
| **运算符** | **逻辑表达式** | **描述**                                               | **实例（a=10，b=20）**     |
| **and** | x and y   | 布尔"与" - 如果 x 为 False，x and y 返回 False，否则它返回 y 的计算值。  | (a and b) 返回 20。      |
| **or**  | x or y    | 布尔"或" - 如果 x 是非 0，它返回 x 的计算值，否则它返回 y 的计算值。           | (a or b) 返回 10。       |
| **not** | not x     | 布尔"非" - 如果 x 为 True，返回 False 。如果 x 为 False，它返回 True。 | not(a and b) 返回 False |

**1.2.5 成员运算符**

|         |                                   |                                    |
| ------- | --------------------------------- | ---------------------------------- |
| **运算符** | **描述**                            | **实例**                             |
| in      | 如果在指定的序列中找到值返回 True，否则返回 False。   | x 在 y 序列中 , 如果 x 在 y 序列中返回 True。   |
| not in  | 如果在指定的序列中没有找到值返回 True，否则返回 False。 | x 不在 y 序列中 , 如果 x 不在 y 序列中返回 True。 |

**1.2.6 身份运算符**

|         |                           |                                                                            |
| ------- | ------------------------- | -------------------------------------------------------------------------- |
| **运算符** | **描述**                    | **实例**                                                                     |
| is      | is 是判断两个标识符是不是引用自一个对象     | **x is y**, 类似 **id(x) == id(y)** , 如果引用的是同一个对象则返回 True，否则返回 False         |
| is not  | is not 是判断两个标识符是不是引用自不同对象 | **x is not y** ， 类似 **id(a) \!= id(b)**。如果引用的不是同一个对象则返回结果 True，否则返回 False。 |

**1.2.7 位运算符**

|          |                                                            |                                                    |
| -------- | ---------------------------------------------------------- | -------------------------------------------------- |
| **运算符**  | **描述**                                                     | **实例（a=60，b=13）**                                  |
| **&**    | 按位与运算符：参与运算的两个值,如果两个相应位都为1,则该位的结果为1,否则为0                   | (a & b) 输出结果 12 ，二进制解释： 0000 1100                  |
| **|**    | 按位或运算符：只要对应的二个二进位有一个为1时，结果位就为1。                            | (a | b) 输出结果 61 ，二进制解释： 0011 1101                  |
| **^**    | 按位异或运算符：当两对应的二进位相异时，结果为1                                   | (a ^ b) 输出结果 49 ，二进制解释： 0011 0001                  |
| **\~**   | 按位取反运算符：对数据的每个二进制位取反,即把1变为0,把0变为1 。**\~x** 类似于 **-x-1**    | (\~a ) 输出结果 -61 ，二进制解释： 1100 0011，在一个有符号二进制数的补码形式。 |
| **\<\<** | 左移动运算符：运算数的各二进位全部左移若干位，由 **\<\<** 右边的数字指定了移动的位数，高位丢弃，低位补0。 | a \<\< 2 输出结果 240 ，二进制解释： 1111 0000                |
| **\>\>** | 右移动运算符：把"\>\>"左边的运算数的各二进位全部右移若干位，**\>\>** 右边的数字指定了移动的位数    | a \>\> 2 输出结果 15 ，二进制解释： 0000 1111                 |

**1.2.8 运算符优先级**

|                              |                                   |
| ---------------------------- | --------------------------------- |
| **运算符**                      | **描述**                            |
| \*\*                         | 指数 (最高优先级)                        |
| \~ + -                       | 按位翻转, 一元加号和减号 (最后两个的方法名为 +@ 和 -@) |
| \* / % //                    | 乘，除，取模和取整除                        |
| \+ -                         | 加法减法                              |
| \>\> \<\<                    | 右移，左移运算符                          |
| &                            | 位 'AND'                           |
| ^ |                          | 位运算符                              |
| \<= \< \> \>=                | 比较运算符                             |
| \<\> == \!=                  | 等于运算符                             |
| \= %= /= //= -= += \*= \*\*= | 赋值运算符                             |
| is is not                    | 身份运算符                             |
| in not in                    | 成员运算符                             |
| not and or                   | 逻辑运算符                             |

**1.2 代码块约束**

同python一样，代码块不使用大括号 **{}** 来控制类，函数以及其他逻辑判断。采用缩进来写模块。

缩进的空白数量是可变的，但是所有代码块语句必须包含相同的缩进空白数量，这个必须严格执行。

**1.3 variable（变量）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例1</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
key <strong>=</strong> value</td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
size = 10</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**1.4 function（函数）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例1</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>def</strong> functionname<strong>(</strong><br />
parameters<br />
<strong>):</strong><br />
function_code<br />
<strong>return [</strong>expression<strong>]</strong></td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td>Python<br />
def print_msg(msg):<br />
print(msg)<br />
return</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**1.5 内置函数**

第4到第9节所涉及的函数皆为内置函数，可直接调用，除此之外还有下述内值函数。

|    |    |    |
| -- | -- | -- |
| 函数 | 功能 | 举例 |
|    |    |    |

**二、类型约束**

**2.1 Numbers（数字）**

**2.1.1 int（有符号整型）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例1</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>key = &lt;int&gt;</strong></td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>size_a = 1</strong><br />
<strong>size_b = -1</strong><br />
<strong>size_c = 0x01</strong><br />
<strong>size_d = -0x01</strong><br />
...</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**2.1.2 float（浮点型）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例1</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>key = &lt;float&gt;</strong></td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>size_a = 0.1</strong><br />
<strong>size_b = -0.1</strong><br />
<strong>size_c = 0.1e+18</strong><br />
<strong>size_d = 0.1</strong><br />
...</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**2.1.3 complex（复数）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例1</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>key = &lt;complex&gt;</strong></td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>size_a = 3.14j</strong><br />
<strong>size_b =</strong> 3e+26J<br />
<strong>size_c =</strong> 0.1e-7j<br />
size_d = -.6545+0J</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**2.2 String（字符串）**

[参考链接](https://docs.python.org/zh-cn/3/tutorial/index.html)

**2.3 List（列表）**

[参考链接](https://www.runoob.com/python/python-lists.html)

**2.4 Tuple（元组）**

[参考链接](https://www.runoob.com/python/python-tuples.html)

**2.5 Dictionary（字典）**

[参考链接](https://www.runoob.com/python/python-dictionary.html)

**三、逻辑控制**

**3.1 if**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if</strong> conditional_expression_A:<br />
function_code_A<br />
<strong>elif</strong> conditional_expression_B:<br />
function_code_B<br />
<strong>else</strong>:<br />
function_code_C</td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if</strong> 1 &lt; 2:<br />
print('1&lt;2')<br />
<strong>elif</strong> 1 &gt; 2:<br />
print('1&gt;2')<br />
<strong>else</strong>:<br />
print('1==2')</td>
</tr>
</tbody>
</table>
<p>将输出：</p>
<table>
<tbody>
<tr class="odd">
<td><br />
1&lt;2</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**3.2 for**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>for</strong> iterating_var <strong>in</strong> sequence<strong>:</strong><br />
function_code</td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td>Perl<br />
<strong>for</strong> now_char <strong>in</strong> 'hello!':<br />
<strong>print(</strong>'当前字符：', <strong>now_char)</strong></td>
</tr>
</tbody>
</table>
<p>将输出：</p>
<table>
<tbody>
<tr class="odd">
<td><br />
当前字符：h<br />
当前字符：e<br />
当前字符：l<br />
当前字符：l<br />
当前字符：o<br />
当前字符：!</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**3.3 while**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>while</strong> conditional_expression<strong>：</strong><br />
function_code</td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td>Swift<br />
now_num = 0<br />
while now_num &lt; 2:<br />
<strong>print(</strong>'当前数字：', now_num<strong>)</strong><br />
now_num = now_num + 1</td>
</tr>
</tbody>
</table>
<p>将输出：</p>
<table>
<tbody>
<tr class="odd">
<td><br />
当前数字：0<br />
当前数字：1</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**3.4 break**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例（借助while说明）</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>break</strong></td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td>Swift<br />
now_num = 0<br />
while true:<br />
<strong>print(</strong>'当前数字：', now_num<strong>)</strong><br />
now_num = now_num + 1<br />
break<br />
<br />
<strong>print(</strong>'最终数字：', now_num<strong>) </strong></td>
</tr>
</tbody>
</table>
<p>将输出：</p>
<table>
<tbody>
<tr class="odd">
<td><br />
当前数字：0<br />
最终数字：1</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**3.5 continue**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>举例（借助while说明）</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>continue</strong></td>
</tr>
</tbody>
</table></td>
<td><table>
<tbody>
<tr class="odd">
<td>Swift<br />
now_num = 0<br />
while now_num &lt; 2:<br />
now_num = now_num + 1<br />
continue<br />
<strong>print(</strong>'当前数字：', now_num<strong>)</strong><br />
<br />
<strong>print(</strong>'最终数字：', now_num<strong>) </strong></td>
</tr>
</tbody>
</table>
<p>将输出：</p>
<table>
<tbody>
<tr class="odd">
<td><br />
最终数字：2</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**四、Cyberdog ability set（铁蛋能力集）**

在此，为了铁蛋能更好的服务客户，我们对铁蛋具备的大部分能力抽象为浅显易懂的功能接口集合，以便用户直接调用，也就是本节的主要内容，在此之前我们对铁蛋通用参数的有效值进行如下约束：

**4.0 铁蛋坐标系**

<table>
<tbody>
<tr class="odd">
<td>右手准则坐标系</td>
<td>右手准则旋转方向</td>
</tr>
<tr class="even">
<td><p>手指指向方向为轴向的正方向，即为：</p>
<p>X轴速度方向：前正后负</p>
<p>Y轴速度方向：左正右负</p>
<p>Z轴速度方向：左正右负</p></td>
<td><p>手指指向方向为轴向的正方向，即为：</p>
<p>X轴旋转方向：左负右正</p>
<p>Y轴旋转方向：上负下正</p>
<p>Z轴旋转方向：左正右负</p></td>
</tr>
<tr class="odd">
<td>欧拉角</td>
<td></td>
</tr>
<tr class="even">
<td><a href="https://quaternions.online/">欧拉角和四元数说明</a></td>
<td></td>
</tr>
</tbody>
</table>

如上所示，铁蛋符合右手准侧坐标系，

**4.1 铁蛋能力集参数约束表**

<table>
<tbody>
<tr class="odd">
<td>序号</td>
<td>分类</td>
<td>参数</td>
<td>变量</td>
<td>单位</td>
<td><p>分辨率</p>
<p>（控制粒度）</p></td>
<td>数值类型</td>
<td>语义类型</td>
<td>最小值</td>
<td>默认</td>
<td>最大值</td>
<td>方向</td>
<td>备注</td>
</tr>
<tr class="even">
<td>1</td>
<td>运动</td>
<td>质心X轴约束</td>
<td>centroid_x</td>
<td>m</td>
<td>0.001m</td>
<td>浮点数</td>
<td><del>绝对(未开放)</del></td>
<td>0</td>
<td>0</td>
<td>0</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td>浮点数</td>
<td>相对</td>
<td>-0.4</td>
<td>0</td>
<td>0.4</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>2</td>
<td></td>
<td>质心Y轴约束</td>
<td>centroid_y</td>
<td></td>
<td></td>
<td>浮点数</td>
<td><del>绝对(未开放)</del></td>
<td>0</td>
<td>0</td>
<td>0</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td>浮点数</td>
<td>相对</td>
<td>-0.3</td>
<td>0</td>
<td>0.3</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>3</td>
<td></td>
<td>质心Z轴约束</td>
<td>zcentroid_z</td>
<td></td>
<td></td>
<td>浮点数</td>
<td>绝对</td>
<td>0.1</td>
<td>0</td>
<td>0.3</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td>浮点数</td>
<td>相对</td>
<td>-0.2</td>
<td>0</td>
<td>0.2</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>4</td>
<td></td>
<td>支点X轴约束</td>
<td>fulcrum_x</td>
<td>m</td>
<td>0.005m</td>
<td>浮点数</td>
<td></td>
<td>-0.5</td>
<td>0</td>
<td>0.5</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td>5</td>
<td></td>
<td>支点Y轴约束</td>
<td>fulcrum_y</td>
<td>m</td>
<td>0.005m</td>
<td>浮点数</td>
<td></td>
<td>-0.5</td>
<td>0</td>
<td>0.5</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>6</td>
<td></td>
<td>支点Z轴约束</td>
<td>fulcrum_z</td>
<td>m</td>
<td>0.005m</td>
<td>浮点数</td>
<td></td>
<td>-0.5</td>
<td>0</td>
<td>0.5</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td>7</td>
<td></td>
<td>机身翻滚：X轴</td>
<td>roll</td>
<td>deg</td>
<td>0.1</td>
<td>浮点数</td>
<td>绝对</td>
<td>-0.45*57.3=-25.8</td>
<td>0</td>
<td>0.45*57.3=25.8</td>
<td>方向：左负右正</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td>浮点数</td>
<td>相对</td>
<td>-0.9*57.3=-51.6</td>
<td>0</td>
<td>0.9*57.3=51.6</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td>8</td>
<td></td>
<td>机身俯仰：Y轴</td>
<td>pitch</td>
<td>deg</td>
<td>0.1</td>
<td>浮点数</td>
<td>绝对</td>
<td>-0.45*57.3=-25.8</td>
<td>0</td>
<td>0.45*57.3=25.8</td>
<td>方向：上负下正</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td>浮点数</td>
<td>相对</td>
<td>-0.9*57.3=-51.6</td>
<td>0</td>
<td>0.9*57.3=51.6</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td>9</td>
<td></td>
<td>机身偏航：Z轴</td>
<td>yaw</td>
<td>deg</td>
<td>0.1</td>
<td>浮点数</td>
<td>绝对</td>
<td>-0.45*57.3=-25.8</td>
<td>0</td>
<td>0.45*57.3=25.8</td>
<td>方向：左正右负</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td>浮点数</td>
<td>相对</td>
<td>-0.9*57.3=-51.6</td>
<td>0</td>
<td>0.9*57.3=51.6</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td>10</td>
<td></td>
<td>x轴向行走速度</td>
<td>x_velocity</td>
<td>m/s</td>
<td>0.01</td>
<td>浮点数</td>
<td></td>
<td>-1.6</td>
<td>0</td>
<td>1.6</td>
<td>方向：前正后负</td>
<td></td>
</tr>
<tr class="even">
<td>11</td>
<td></td>
<td>y轴向行走速度</td>
<td>y_velocity</td>
<td>m/s</td>
<td>0.01</td>
<td>浮点数</td>
<td></td>
<td>-1.2</td>
<td>0</td>
<td>1.2</td>
<td>方向：左正右负</td>
<td></td>
</tr>
<tr class="odd">
<td>12</td>
<td></td>
<td>z轴向行走速度</td>
<td>z_velocity</td>
<td>deg/s</td>
<td>0.1</td>
<td>浮点数</td>
<td></td>
<td>-2.5*57.3=-114.6</td>
<td>0</td>
<td>2.5*57.3= 114.6</td>
<td>方向：左正右负</td>
<td></td>
</tr>
<tr class="even">
<td>13</td>
<td></td>
<td>x轴向跳跃速度</td>
<td>x_jump_velocity</td>
<td>m/s</td>
<td>0.01</td>
<td>浮点数</td>
<td></td>
<td>-0.25</td>
<td>0</td>
<td>0.15</td>
<td>方向：前正后负</td>
<td></td>
</tr>
<tr class="odd">
<td>14</td>
<td></td>
<td>y轴向跳跃速度</td>
<td>y_jump_velocity</td>
<td>m/s</td>
<td>0.01</td>
<td>浮点数</td>
<td></td>
<td>-0.1</td>
<td>0</td>
<td>0.1</td>
<td>方向：左正右负</td>
<td></td>
</tr>
<tr class="even">
<td>15</td>
<td></td>
<td>z轴向跳跃速度</td>
<td>z_jump_velocity</td>
<td>deg/s</td>
<td>0.1</td>
<td>浮点数</td>
<td></td>
<td>-0.5* 57.3=-28.65</td>
<td>0</td>
<td>0.5* 57.3=28.65</td>
<td>方向：左正右负</td>
<td></td>
</tr>
<tr class="odd">
<td>16</td>
<td></td>
<td>前腿抬腿高度</td>
<td>front_leg_lift</td>
<td>m</td>
<td>0.001</td>
<td>浮点数</td>
<td></td>
<td>0.005</td>
<td>0.03</td>
<td>0.1</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>17</td>
<td></td>
<td>前腿抬腿高度</td>
<td>back_leg_lift</td>
<td>m</td>
<td>0.001</td>
<td>浮点数</td>
<td></td>
<td>0.005</td>
<td>0.03</td>
<td>0.1</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td>18</td>
<td></td>
<td>期望距离</td>
<td>distance</td>
<td>m</td>
<td>0.01</td>
<td>浮点数</td>
<td></td>
<td>0</td>
<td>0</td>
<td>10</td>
<td></td>
<td>0:字段无效</td>
</tr>
<tr class="even">
<td>19</td>
<td></td>
<td>期望耗时</td>
<td>duration</td>
<td>s</td>
<td>2</td>
<td>浮点数</td>
<td></td>
<td>0</td>
<td>1</td>
<td>6</td>
<td></td>
<td>0:字段无效</td>
</tr>
<tr class="odd">
<td>20</td>
<td>语音</td>
<td>音量</td>
<td>volume</td>
<td>%</td>
<td>1</td>
<td>整型</td>
<td></td>
<td>0</td>
<td>-1</td>
<td>100</td>
<td>值越大音量越大</td>
<td>-1: 字段无效</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
</tr>
</tbody>
</table>

**4.2 铁蛋能力集通用类型约束表**

**[Time](https://docs.ros2.org/latest/api/builtin_interfaces/msg/Time.html)：时间戳**

**[Header](https://docs.ros2.org/latest/api/std_msgs/msg/Header.html)：帧头**

**[LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html)：雷达消息**

**[Range](https://docs.ros2.org/latest/api/sensor_msgs/msg/Range.html)：超声波消息**

**[Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)：里程计消息**

**[Imu](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)：惯导消息**

**[Point](https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html)：点消息**

**[Quaternion](https://docs.ros2.org/latest/api/geometry_msgs/msg/Quaternion.html)：四元数消息**

**[Pose](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html)：惯导消息**

**[Vector3](https://docs.ros2.org/latest/api/geometry_msgs/msg/Vector3.html)：惯导消息**

**[Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)：惯导消息**

**[PoseWithCovariance](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseWithCovariance.html)：惯导消息**

**[TwistWithCovariance](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistWithCovariance.html)：惯导消息**

**BmsStatus：电池管理系统消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td><a href="https://docs.ros2.org/latest/api/std_msgs/msg/Header.html">BmsStatus</a></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>header</td>
<td><a>Header</a></td>
<td>消息头</td>
</tr>
<tr class="odd">
<td></td>
<td>batt_volt</td>
<td>int</td>
<td>电压 - mV</td>
</tr>
<tr class="even">
<td></td>
<td>batt_curr</td>
<td>int</td>
<td>电流 - mA</td>
</tr>
<tr class="odd">
<td></td>
<td>batt_soc</td>
<td>int</td>
<td>剩余电量</td>
</tr>
<tr class="even">
<td></td>
<td>batt_temp</td>
<td>int</td>
<td>温度 - C</td>
</tr>
<tr class="odd">
<td></td>
<td>batt_st</td>
<td>int</td>
<td>电池模式:bit0 - 正常模式; bit1 - 正在充电; bit2 - 充电完成; bit3 - 电机掉电; bit4 - 软关机</td>
</tr>
<tr class="even">
<td></td>
<td>key_val</td>
<td>int</td>
<td>关机信号:1 - 关机; 0 - 正常/不关机</td>
</tr>
<tr class="odd">
<td></td>
<td>disable_charge</td>
<td>int</td>
<td>禁用充电</td>
</tr>
<tr class="even">
<td></td>
<td>power_supply</td>
<td>int</td>
<td>电源</td>
</tr>
<tr class="odd">
<td></td>
<td>buzze</td>
<td>int</td>
<td>蜂鸣</td>
</tr>
<tr class="even">
<td></td>
<td>status</td>
<td>int</td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>batt_health</td>
<td>int</td>
<td>电池健康</td>
</tr>
<tr class="even">
<td></td>
<td>batt_loop_number</td>
<td>int</td>
<td>电池循环数</td>
</tr>
<tr class="odd">
<td></td>
<td>powerboard_status</td>
<td>int</td>
<td>电源板状态: bit0 - 串口错误{1 - 有错误; 0 - 无错误}</td>
</tr>
<tr class="even">
<td></td>
<td>power_normal</td>
<td>bool</td>
<td>正常模式</td>
</tr>
<tr class="odd">
<td></td>
<td>power_wired_charging</td>
<td>bool</td>
<td>有线充电中</td>
</tr>
<tr class="even">
<td></td>
<td>power_finished_charging</td>
<td>bool</td>
<td>充电完成</td>
</tr>
<tr class="odd">
<td></td>
<td>power_motor_shutdown</td>
<td>bool</td>
<td>电机掉电</td>
</tr>
<tr class="even">
<td></td>
<td>power_soft_shutdown</td>
<td>bool</td>
<td>软关机</td>
</tr>
<tr class="odd">
<td></td>
<td>power_wp_place</td>
<td>bool</td>
<td>无线充电在位</td>
</tr>
<tr class="even">
<td></td>
<td>power_wp_charging</td>
<td>bool</td>
<td>无线充电中</td>
</tr>
<tr class="odd">
<td></td>
<td>power_expower_supply</td>
<td>bool</td>
<td>外部供电</td>
</tr>
</tbody>
</table>

**TouchStatus：触摸板消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td><a href="https://git.n.xiaomi.com/MiRoboticsLab/rop/bridges/-/blob/dev/protocol/ros/msg/TouchStatus.msg">TouchStatus</a></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>header</td>
<td><a>Header</a></td>
<td>消息头</td>
</tr>
<tr class="odd">
<td></td>
<td>touch_state</td>
<td>int</td>
<td>触摸板状态{1:单击,2：双击,3：长按}</td>
</tr>
</tbody>
</table>

**GpsPayload：全球定位系统消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td><a href="https://git.n.xiaomi.com/MiRoboticsLab/rop/bridges/-/blob/dev/protocol/ros/msg/GpsPayload.msg">GpsPayload</a></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>sec</td>
<td>int</td>
<td>秒</td>
</tr>
<tr class="odd">
<td></td>
<td>nanosec</td>
<td>int</td>
<td>纳秒</td>
</tr>
<tr class="even">
<td></td>
<td>itow</td>
<td>int</td>
<td>GPS时间戳</td>
</tr>
<tr class="odd">
<td></td>
<td>fix_type</td>
<td>int</td>
<td>GNSS类型</td>
</tr>
<tr class="even">
<td></td>
<td>num_sv</td>
<td>int</td>
<td>当前搜星卫星数量</td>
</tr>
<tr class="odd">
<td></td>
<td>lon</td>
<td>int</td>
<td>经度</td>
</tr>
<tr class="even">
<td></td>
<td>lat</td>
<td>int</td>
<td>纬度</td>
</tr>
</tbody>
</table>

**SingleTofPayload：单个Tof数据消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td><a href="https://git.n.xiaomi.com/MiRoboticsLab/rop/bridges/-/blob/dev/protocol/ros/msg/SingleTofPayload.msg">SingleTofPayload</a></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>header</td>
<td><a>Header</a></td>
<td>消息头</td>
</tr>
<tr class="odd">
<td></td>
<td>data_available</td>
<td>bool</td>
<td>数据是否可用</td>
</tr>
<tr class="even">
<td></td>
<td>tof_position</td>
<td>int</td>
<td>传感器的位置(左前:0,右前:1,左后:2,右后:3)</td>
</tr>
<tr class="odd">
<td></td>
<td>data</td>
<td>float</td>
<td>传感器数据[m]</td>
</tr>
</tbody>
</table>

**HeadTofPayload：头部Tof数据消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td><a href="https://git.n.xiaomi.com/MiRoboticsLab/rop/bridges/-/blob/dev/protocol/ros/msg/HeadTofPayload.msg">HeadTofPayload</a></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>left_head</td>
<td><a>SingleTofPayload</a></td>
<td>头部左侧Tof</td>
</tr>
<tr class="odd">
<td></td>
<td>right_head</td>
<td><a>SingleTofPayload</a></td>
<td>头部右侧Tof</td>
</tr>
</tbody>
</table>

**RearTofPayload：尾部Tof数据消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td><a href="https://git.n.xiaomi.com/MiRoboticsLab/rop/bridges/-/blob/dev/protocol/ros/msg/RearTofPayload.msg">RearTofPayload</a></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>left_rear</td>
<td><a>SingleTofPayload</a></td>
<td>尾部左侧Tof</td>
</tr>
<tr class="odd">
<td></td>
<td>right_rear</td>
<td><a>SingleTofPayload</a></td>
<td>尾部右侧Tof</td>
</tr>
</tbody>
</table>

**TofPayload：激光测距消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>TofPayload</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>head</td>
<td><a>HeadTofPayload</a></td>
<td>头部tof</td>
</tr>
<tr class="odd">
<td></td>
<td>rear</td>
<td><a>RearTofPayload</a></td>
<td>尾部tof</td>
</tr>
</tbody>
</table>

**StateCode：状态码**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>枚举</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>StateCode</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合</p>
<p>法</p>
<p>值</p>
<p>约</p>
<p>束</p></td>
<td>键</td>
<td>值</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>invalid</td>
<td>-1</td>
<td>无效</td>
</tr>
<tr class="odd">
<td></td>
<td>success</td>
<td>0x00</td>
<td>成功</td>
</tr>
<tr class="even">
<td></td>
<td>fail</td>
<td>0x01</td>
<td>失败</td>
</tr>
<tr class="odd">
<td></td>
<td>no_data_update</td>
<td>0x20</td>
<td>无数据更新</td>
</tr>
<tr class="even">
<td></td>
<td>command_waiting_execute</td>
<td>0x30</td>
<td>待执行时发生错误</td>
</tr>
<tr class="odd">
<td></td>
<td>service_client_interrupted</td>
<td>0x40</td>
<td>客户端在请求服务出现时被打断</td>
</tr>
<tr class="even">
<td></td>
<td>service_appear_timeout</td>
<td>0x41</td>
<td>等待服务出现（启动）超时</td>
</tr>
<tr class="odd">
<td></td>
<td>service_request_interrupted</td>
<td>0x42</td>
<td>请求服务中断</td>
</tr>
<tr class="even">
<td></td>
<td>service_request_timeout</td>
<td>0x43</td>
<td>请求服务超时/延迟</td>
</tr>
<tr class="odd">
<td></td>
<td>spin_future_interrupted</td>
<td>0x50</td>
<td>请求服务中断</td>
</tr>
<tr class="even">
<td></td>
<td>spin_future_timeout</td>
<td>0x51</td>
<td>请求服务超时/延迟</td>
</tr>
</tbody>
</table>

**State：状态**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>State</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>code</td>
<td><a>StateCode</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>describe</td>
<td>string</td>
<td>状态描述</td>
</tr>
</tbody>
</table>

**DefaultAndMaximum：运动参数默认类型**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>DefaultAndMaximum</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>minimum_value</td>
<td>double</td>
<td>最小值</td>
</tr>
<tr class="odd">
<td></td>
<td>default_value</td>
<td>double</td>
<td>默认值</td>
</tr>
<tr class="even">
<td></td>
<td>maximum_value</td>
<td>double</td>
<td>最大值</td>
</tr>
<tr class="odd">
<td></td>
<td>unit</td>
<td>string</td>
<td>单位</td>
</tr>
</tbody>
</table>

**MotionParams：运动参数消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MotionParams</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>centroid_x</td>
<td><a>DefaultAndMaximum</a></td>
<td>质心X轴约束</td>
</tr>
<tr class="odd">
<td></td>
<td>centroid_y</td>
<td><a>DefaultAndMaximum</a></td>
<td>质心Y轴约束</td>
</tr>
<tr class="even">
<td></td>
<td>centroid_z</td>
<td><a>DefaultAndMaximum</a></td>
<td>质心Z轴约束</td>
</tr>
<tr class="odd">
<td></td>
<td>fulcrum_x</td>
<td><a>DefaultAndMaximum</a></td>
<td>支点X轴约束</td>
</tr>
<tr class="even">
<td></td>
<td>fulcrum_y</td>
<td><a>DefaultAndMaximum</a></td>
<td>支点Y轴约束</td>
</tr>
<tr class="odd">
<td></td>
<td>fulcrum_z</td>
<td><a>DefaultAndMaximum</a></td>
<td>支点Z轴约束</td>
</tr>
<tr class="even">
<td></td>
<td>roll</td>
<td><a>DefaultAndMaximum</a></td>
<td>机身翻滚</td>
</tr>
<tr class="odd">
<td></td>
<td>pitch</td>
<td><a>DefaultAndMaximum</a></td>
<td>机身俯仰</td>
</tr>
<tr class="even">
<td></td>
<td>yaw</td>
<td><a>DefaultAndMaximum</a></td>
<td>机身偏航</td>
</tr>
<tr class="odd">
<td></td>
<td>x_velocity</td>
<td><a>DefaultAndMaximum</a></td>
<td>X轴速度</td>
</tr>
<tr class="even">
<td></td>
<td>y_velocity</td>
<td><a>DefaultAndMaximum</a></td>
<td>Y轴速度</td>
</tr>
<tr class="odd">
<td></td>
<td>z_velocity</td>
<td><a>DefaultAndMaximum</a></td>
<td>Z轴角速度</td>
</tr>
<tr class="even">
<td></td>
<td>front_leg_lift</td>
<td><a>DefaultAndMaximum</a></td>
<td>前腿抬腿高度</td>
</tr>
<tr class="odd">
<td></td>
<td>back_leg_lift</td>
<td><a>DefaultAndMaximum</a></td>
<td>后腿抬腿高度</td>
</tr>
<tr class="even">
<td></td>
<td>distance</td>
<td><a>DefaultAndMaximum</a></td>
<td>期望距离</td>
</tr>
<tr class="odd">
<td></td>
<td>duration</td>
<td><a>DefaultAndMaximum</a></td>
<td>期望时间</td>
</tr>
<tr class="even">
<td></td>
<td>delta</td>
<td><a>DefaultAndMaximum</a></td>
<td>变化量</td>
</tr>
</tbody>
</table>

**MotionResultServiceResponse：运动结果服务反馈**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MotionResultServiceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>motion_id</td>
<td>int32</td>
<td>机器人运控姿态状态</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td>result</td>
<td>bool</td>
<td>执行结果</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td>code</td>
<td>int32</td>
<td>标准错误码</td>
</tr>
</tbody>
</table>

**MotionServoCmdResponse：运动伺服指令反馈**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MotionServoCmdResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>motion_id</td>
<td>int32</td>
<td>机器人运控姿态状态</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td>result</td>
<td>bool</td>
<td>执行结果</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td>code</td>
<td>int32</td>
<td>标准错误码</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td>order_process_bar</td>
<td>int8</td>
<td>订单流程</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td>status</td>
<td>int32</td>
<td>状态</td>
</tr>
</tbody>
</table>

**MotionSequenceServiceResponse：序列运动服务反馈**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MotionSequenceServiceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>result</td>
<td>bool</td>
<td>执行结果</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td>code</td>
<td>int32</td>
<td>状态码</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td>describe</td>
<td>string</td>
<td>状态码描述</td>
</tr>
</tbody>
</table>

**LedConstraint ：LED约束**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td></td>
<td></td>
<td>枚举</td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td></td>
<td></td>
<td>LedConstraint</td>
<td></td>
</tr>
<tr class="odd">
<td><p>合</p>
<p>法</p>
<p>值</p>
<p>约</p>
<p>束</p></td>
<td>场景</td>
<td></td>
<td>键值</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>控制目标</td>
<td></td>
<td>target_head</td>
<td>头灯</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>target_tail</td>
<td>尾灯</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>target_mini</td>
<td>眼灯</td>
</tr>
<tr class="odd">
<td></td>
<td><p>头</p>
<p>尾</p>
<p>灯</p>
<p>带</p></td>
<td><p>灯效</p>
<p>（支持RGB调色）</p></td>
<td>effect_line_on</td>
<td>常亮</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>effect_line_blink</td>
<td>闪烁</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>effect_line_blink_fast</td>
<td>快速闪烁</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>effect_line_breath</td>
<td>呼吸</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>effect_line_breath_fast</td>
<td>快速呼吸</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>effect_line_one_by_one</td>
<td>逐个点亮</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>effect_line_one_by_one_fast</td>
<td>快速逐个点亮</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td><p>系统灯效</p>
<p>（不支持RGB调色）</p></td>
<td>system_effect_line_off</td>
<td>常灭</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_red_on</td>
<td>红灯常亮</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_red_blink</td>
<td>红灯闪烁</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_red_blink_fast</td>
<td>红灯快速闪烁</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_red_breath</td>
<td>红灯呼吸</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_red_breath_fast</td>
<td>红灯快速呼吸</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_red_one_by_one</td>
<td>红灯逐个点亮</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_red_one_by_one_fast</td>
<td>红灯快速逐个点亮</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_blue_on</td>
<td>蓝灯常亮</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_blue_blink</td>
<td>蓝灯闪烁</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_blue_blink_fast</td>
<td>蓝灯快速闪烁</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_blue_breath</td>
<td>蓝灯呼吸</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_blue_breath_fast</td>
<td>蓝灯快速呼吸</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_blue_one_by_one</td>
<td>蓝灯逐个点亮</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_blue_one_by_one_fast</td>
<td>蓝灯快速逐个点亮</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_yellow_on</td>
<td>黄灯常亮</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_ yellow_blink</td>
<td>黄灯闪烁</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_yellow_blink_fast</td>
<td>黄灯快速闪烁</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_yellow_breath</td>
<td>黄灯呼吸</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_yellow_breath_fast</td>
<td>黄灯快速呼吸</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_yellow_one_by_one</td>
<td>黄灯逐个点亮</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_line_yellow_one_by_one_fast</td>
<td>黄灯快速逐个点亮</td>
</tr>
<tr class="even">
<td></td>
<td><p>眼</p>
<p>灯</p></td>
<td><p>灯效</p>
<p>（支持RGB调色）</p></td>
<td>effect_mini_circular_breath</td>
<td>圆形缩放</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>effect_mini_circular_ring</td>
<td>画圆环</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td><p>系统灯效</p>
<p>（不支持RGB调色）</p></td>
<td>system_effect_mini_off</td>
<td>常灭</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_mini_rectangle_color</td>
<td>方块变色</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_mini_centre_color</td>
<td>中间彩带</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>system_effect_mini_three_circular</td>
<td>三圆呼吸</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>system_effect_mini_one_by_one</td>
<td>彩带逐个点亮</td>
</tr>
</tbody>
</table>

**LedSeviceResponse：LED服务反馈**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>LedSeviceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>code</td>
<td>int32</td>
<td><p>标准错误码</p>
<table>
<tbody>
<tr class="odd">
<td>C++<br />
<br />
int32 SUCCEED =0 # 当前请求参数合理，优先级最高，请求灯效执行成功<br />
int32 TIMEOUT =1107 # 当前请求led硬件响应超时<br />
int32 TARGET_ERROR =1121 # 当前请求的target参数为空或者不在可选列表中<br />
int32 PRIORITY_ERROR =1122 # 当前请求的client为空或者不在预设的优先级列表中<br />
int32 MODE_ERROR = 1123 # 当前请求的mode参数为空或者不在可选列表中<br />
int32 EFFECT_ERROR =1124 # 当前请求的effect参数为空或者不在可选列表中<br />
int32 LOW_PRIORITY = 1125 # 当前请求优先级较低，无法立即执行请求灯效</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**ConnectorStatus：网络连接消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td><a href="https://git.n.xiaomi.com/MiRoboticsLab/rop/bridges/-/blob/dev/protocol/ros/msg/ConnectorStatus.msg">ConnectorStatus</a></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>is_connected</td>
<td>bool</td>
<td>是否连接wifi</td>
</tr>
<tr class="odd">
<td></td>
<td>is_internet</td>
<td>bool</td>
<td>是否可以访问外网</td>
</tr>
<tr class="even">
<td></td>
<td>ssid</td>
<td>string</td>
<td>wifi名称</td>
</tr>
<tr class="odd">
<td></td>
<td>robot_ip</td>
<td>string</td>
<td>机器人IP</td>
</tr>
<tr class="even">
<td></td>
<td>provider_ip</td>
<td>string</td>
<td>wifi提供方/移动端 IP</td>
</tr>
<tr class="odd">
<td></td>
<td>strength</td>
<td>int</td>
<td>wifi信号强度</td>
</tr>
<tr class="even">
<td></td>
<td>code</td>
<td>int</td>
<td>标准错误码</td>
</tr>
</tbody>
</table>

**Vector\<xxx\>：列表**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>列表</td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>vector&lt;xxx&gt;</td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>可用操作</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>接口</td>
<td>说明</td>
</tr>
<tr class="odd">
<td></td>
<td>empty();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>size();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>max_size();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>capacity();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>clear();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>push_back();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>pop_back();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>at();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>front();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>back();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
</tr>
</tbody>
</table>

**MotionSequenceGait：序列步态消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MotionSequenceGait</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>right_forefoot</td>
<td>bool</td>
<td>右前足: 是否接触地面? （默认值：True）</td>
</tr>
<tr class="odd">
<td></td>
<td>left_forefoot</td>
<td>bool</td>
<td>左前足: 是否接触地面? （默认值：True）</td>
</tr>
<tr class="even">
<td></td>
<td>right_hindfoot</td>
<td>bool</td>
<td>右后足: 是否接触地面? （默认值：True）</td>
</tr>
<tr class="odd">
<td></td>
<td>left_hindfoot</td>
<td>bool</td>
<td>左后足: 是否接触地面? （默认值：True）</td>
</tr>
<tr class="even">
<td></td>
<td>duration</td>
<td>int</td>
<td>当前步态持续时间(毫秒) （默认值：1000）</td>
</tr>
</tbody>
</table>

**MotionSequencePace：序列步伐消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MotionSequencePace</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合</p>
<p>法</p>
<p>字</p>
<p>段</p>
<p>约</p>
<p>束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>twist</td>
<td><a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html">geometry_msgs/msg/Twist.msg</a></td>
<td><p>速度（默认值：全0.0）</p>
<table>
<tbody>
<tr class="odd">
<td>C++<br />
linear.x // X轴线速度（m/s）<br />
linear.y // Y轴线速度（m/s）<br />
linear.z // z轴角速度（°/s）</td>
</tr>
</tbody>
</table></td>
</tr>
<tr class="odd">
<td></td>
<td>centroid</td>
<td><a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html">geometry_msgs/msg/Pose.msg</a></td>
<td><p>质心（默认值：全0.0）</p>
<table>
<tbody>
<tr class="odd">
<td>C++<br />
position.x // X轴质心偏移（m）<br />
position.y // Y轴质心偏移（m）<br />
position.z // Z轴质心高度（m）<br />
orientation.x // X轴姿态：R(°)<br />
orientation.y // Y轴姿态：P（°）<br />
orientation.z // Z轴姿态：Y（°）</td>
</tr>
</tbody>
</table></td>
</tr>
<tr class="even">
<td></td>
<td>weight</td>
<td><a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html">geometry_msgs/msg/Twist.msg</a></td>
<td><p>6自由度权重</p>
<table>
<tbody>
<tr class="odd">
<td>C++<br />
linear.x // X轴权重<br />
linear.y // Y轴权重<br />
linear.z // Z轴权重<br />
angular.x // X-R轴权重<br />
angular.y // Y-P轴权重<br />
angular.z // Z-Y轴权重</td>
</tr>
</tbody>
</table></td>
</tr>
<tr class="odd">
<td></td>
<td>right_forefoot</td>
<td><a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/Quaternion.html">geometry_msgs/msg/Quaternion.msg</a></td>
<td><p>右前足信息（默认值：全0.0）</p>
<table>
<tbody>
<tr class="odd">
<td>C++<br />
x // 落脚点位置：X轴偏移量(m)<br />
y // 落脚点位置：Y轴偏移量(m)<br />
z // 落脚点位置：Z轴偏移量(m)<br />
w // 抬脚高度(m)</td>
</tr>
</tbody>
</table></td>
</tr>
<tr class="even">
<td></td>
<td>left_forefoot</td>
<td><a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html">geometry_msgs/msg/Quaternion.msg</a></td>
<td><p>左前足信息（默认值：全0.0）</p>
<table>
<tbody>
<tr class="odd">
<td>C++<br />
x // 落脚点位置：X轴偏移量(m)<br />
y // 落脚点位置：Y轴偏移量(m)<br />
z // 落脚点位置：Z轴偏移量(m)<br />
w // 抬脚高度(m)</td>
</tr>
</tbody>
</table></td>
</tr>
<tr class="odd">
<td></td>
<td>right_hindfoot</td>
<td><a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html">geometry_msgs/msg/Quaternion.msg</a></td>
<td><p>右后足信息（默认值：全0.0）</p>
<table>
<tbody>
<tr class="odd">
<td>C++<br />
x // 落脚点位置：X轴偏移量(m)<br />
y // 落脚点位置：Y轴偏移量(m)<br />
z // 落脚点位置：Z轴偏移量(m)<br />
w // 抬脚高度(m)</td>
</tr>
</tbody>
</table></td>
</tr>
<tr class="even">
<td></td>
<td>left_hindfoot</td>
<td><a href="https://docs.ros2.org/latest/api/geometry_msgs/msg/Point.html">geometry_msgs/msg/Quaternion.msg</a></td>
<td><p>左后足信息（默认值：全0.0）</p>
<table>
<tbody>
<tr class="odd">
<td>C++<br />
x // 落脚点位置：X轴偏移量(m)<br />
y // 落脚点位置：Y轴偏移量(m)<br />
z // 落脚点位置：Z轴偏移量(m)<br />
w // 抬脚高度(m)</td>
</tr>
</tbody>
</table></td>
</tr>
<tr class="odd">
<td></td>
<td>use_mpc_track</td>
<td>bool</td>
<td>是否使用 MPC 轨迹</td>
</tr>
<tr class="even">
<td></td>
<td>landing_gain</td>
<td>float</td>
<td>落地系数[0,1],默认值:1.0</td>
</tr>
<tr class="odd">
<td></td>
<td>friction_coefficient</td>
<td>float</td>
<td>摩擦系数[0.1 1.0] （默认值：0.8）</td>
</tr>
<tr class="even">
<td></td>
<td>duration</td>
<td>int</td>
<td>当前参数持续时间(毫秒)（默认值：1000）</td>
</tr>
</tbody>
</table>

**MotionSequence：运动序列消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MotionSequence</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>name</td>
<td>string</td>
<td>[必须] 运动序列名称（符合变量命名规则）</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>describe</td>
<td>string</td>
<td>运动序列描述</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>gait_list</td>
<td><a>MotionSequenceGait</a>&gt;</td>
<td>步态列表</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>pace_list</td>
<td><a>MotionSequencePace</a>&gt;</td>
<td>步伐列表</td>
<td></td>
</tr>
</tbody>
</table>

**AudioPlaySeviceResponse：语音播放服务反馈消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>AudioPlaySeviceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>status</td>
<td>int32</td>
<td>0播放完毕，1播放失败</td>
</tr>
</tbody>
</table>

**AudioGetVolumeSeviceResponse：语音获取音量服务反馈消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>AudioGetVolumeSeviceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>volume</td>
<td>int32</td>
<td>当前播放语音的音量</td>
</tr>
</tbody>
</table>

**AudioSetVolumeSeviceResponse：语音设置音量服务反馈消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>AudioSetVolumeSeviceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>success</td>
<td>bool</td>
<td>设置音量是否成功</td>
</tr>
</tbody>
</table>

**FaceRecognitionResult：人脸识别结果信息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>FaceRecognitionResult</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>result</td>
<td>int</td>
<td>人员结果</td>
</tr>
<tr class="odd">
<td></td>
<td>username</td>
<td>string</td>
<td>人员名称</td>
</tr>
<tr class="even">
<td></td>
<td>age</td>
<td>float</td>
<td>人员年龄</td>
</tr>
<tr class="odd">
<td></td>
<td>emotion</td>
<td>float</td>
<td>人员情绪</td>
</tr>
</tbody>
</table>

**FaceRecognizedSeviceResponse：人脸识别反馈消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>FaceRecognizedSeviceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td><a>State</a></td>
<td>状态</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>list</td>
<td>list&lt;<a>FaceRecognitionResult</a>&gt;</td>
<td>人脸识别列表</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>可用操作</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>接口</td>
<td>说明</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>empty();</td>
<td>详情参见<a>列表</a></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>size();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>max_size();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>capacity();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>clear();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>append();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>pop();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>at();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>front();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>back();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>dictionary</td>
<td>dictionary&lt;string, <a>FaceRecognitionResult</a>&gt;</td>
<td>人脸识别字典</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>可用操作</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>接口</td>
<td>说明</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>empty();</td>
<td>详情参见<a>字典</a></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>size();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>max_size();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>clear();</td>
<td></td>
</tr>
</tbody>
</table>

**VoiceprintRecognized：声纹识别消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>VoiceprintRecognized</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>list</td>
<td>vector &lt;string&gt;</td>
<td>识别到的人员列表</td>
</tr>
<tr class="even">
<td></td>
<td>data</td>
<td>bool</td>
<td>识别是否成功</td>
</tr>
</tbody>
</table>

**MsgPreset：预置点信息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MsgPreset</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>label_name</td>
<td>string</td>
<td>预置点名称</td>
</tr>
<tr class="odd">
<td></td>
<td>physic_x</td>
<td>float</td>
<td>X轴坐标</td>
</tr>
<tr class="even">
<td></td>
<td>physic_y</td>
<td>float</td>
<td>Y轴坐标</td>
</tr>
</tbody>
</table>

**MapPresetSeviceResponse：地图预置点服务反馈**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MapPresetSeviceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td><a>State</a></td>
<td>状态</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>map_name</td>
<td>string</td>
<td>地图名</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>is_outdoor</td>
<td>bool</td>
<td>是否为室外地图</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>list</td>
<td>list&lt;<a>MsgPreset</a>&gt;</td>
<td>人脸识别列表</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>可用操作</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>接口</td>
<td>说明</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>empty();</td>
<td>详情参见<a>列表</a></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>size();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>max_size();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>capacity();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>clear();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>append();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>pop();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>at();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>front();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>back();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>dictionary</td>
<td>dictionary&lt;string, <a>MsgPreset</a>&gt;</td>
<td>人脸识别字典</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>可用操作</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>接口</td>
<td>说明</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>empty();</td>
<td>详情参见<a>字典</a></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>size();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td>max_size();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td>clear();</td>
<td></td>
</tr>
</tbody>
</table>

**NavigationActionResponse：导航动作反馈**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>NavigationActionResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>result</td>
<td>int</td>
<td><p>服务动作反馈：</p>
<p>0: 成功；</p>
<p>1: 接受；</p>
<p>2: 不可用；</p>
<p>3: 失败；</p>
<p>4: 拒绝；</p>
<p>5: 取消。</p></td>
</tr>
</tbody>
</table>

**GestureType：手势识别类型**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>枚举</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>GestureType</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合</p>
<p>法</p>
<p>值</p>
<p>约</p>
<p>束</p></td>
<td>键</td>
<td>值</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>no_gesture</td>
<td>0</td>
<td>无手势</td>
</tr>
<tr class="odd">
<td></td>
<td>pulling_hand_or_two_fingers_in</td>
<td>1</td>
<td>手掌拉近</td>
</tr>
<tr class="even">
<td></td>
<td>pushing_hand_or_two_fingers_away</td>
<td>2</td>
<td>手掌推开</td>
</tr>
<tr class="odd">
<td></td>
<td>sliding_hand_or_two_fingers_up</td>
<td>3</td>
<td>手向上抬</td>
</tr>
<tr class="even">
<td></td>
<td>sliding_hand_or_two_fingers_down</td>
<td>4</td>
<td>手向下压</td>
</tr>
<tr class="odd">
<td></td>
<td>sliding_hand_or_two_fingers_left</td>
<td>5</td>
<td>手向左推</td>
</tr>
<tr class="even">
<td></td>
<td>sliding_hand_or_two_fingers_right</td>
<td>6</td>
<td>手向右推</td>
</tr>
<tr class="odd">
<td></td>
<td>stop_sign</td>
<td>7</td>
<td>停止手势</td>
</tr>
<tr class="even">
<td></td>
<td>thumb_down</td>
<td>8</td>
<td>大拇指朝下</td>
</tr>
<tr class="odd">
<td></td>
<td>thumb_up</td>
<td>9</td>
<td>大拇指朝上</td>
</tr>
<tr class="even">
<td></td>
<td>zooming_in_with_hand_or_two_fingers</td>
<td>10</td>
<td>张开手掌或手指</td>
</tr>
<tr class="odd">
<td></td>
<td>zooming_out_with_hand_or_two_fingers</td>
<td>11</td>
<td>闭合手掌或手指</td>
</tr>
</tbody>
</table>

**GestureData：手势数据**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>GestureType</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>pulling_hand_or_two_fingers_in</td>
<td>bool</td>
<td>手掌拉近</td>
</tr>
<tr class="odd">
<td></td>
<td>pushing_hand_or_two_fingers_away</td>
<td>bool</td>
<td>手掌推开</td>
</tr>
<tr class="even">
<td></td>
<td>sliding_hand_or_two_fingers_up</td>
<td>bool</td>
<td>手向上抬</td>
</tr>
<tr class="odd">
<td></td>
<td>sliding_hand_or_two_fingers_down</td>
<td>bool</td>
<td>手向下压</td>
</tr>
<tr class="even">
<td></td>
<td>sliding_hand_or_two_fingers_left</td>
<td>bool</td>
<td>手向左推</td>
</tr>
<tr class="odd">
<td></td>
<td>sliding_hand_or_two_fingers_right</td>
<td>bool</td>
<td>手向右推</td>
</tr>
<tr class="even">
<td></td>
<td>stop_sign</td>
<td>bool</td>
<td>停止手势</td>
</tr>
<tr class="odd">
<td></td>
<td>thumb_down</td>
<td>bool</td>
<td>大拇指朝下</td>
</tr>
<tr class="even">
<td></td>
<td>thumb_up</td>
<td>bool</td>
<td>大拇指朝上</td>
</tr>
<tr class="odd">
<td></td>
<td>zooming_in_with_hand_or_two_fingers</td>
<td>bool</td>
<td>张开手掌或手指</td>
</tr>
<tr class="even">
<td></td>
<td>zooming_out_with_hand_or_two_fingers</td>
<td>bool</td>
<td>闭合手掌或手指</td>
</tr>
</tbody>
</table>

**GestureRecognizedSeviceResponse：手势识别服务反馈**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>GestureRecognizedSeviceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>code</td>
<td>int</td>
<td>服务请求状态，0:成功；1:失败。</td>
</tr>
</tbody>
</table>

**GestureRecognizedMessageResponse：手势识别消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>GestureRecognizedMessageResponse</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>data</td>
<td><a>GestureData</a></td>
<td>手势识别状态数据</td>
</tr>
</tbody>
</table>

**SkeletonType：骨骼点识别类型**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>枚举</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>SkeletonType</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合</p>
<p>法</p>
<p>值</p>
<p>约</p>
<p>束</p></td>
<td>键</td>
<td>值</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>squat</td>
<td>1</td>
<td>深蹲</td>
</tr>
<tr class="odd">
<td></td>
<td>highknees</td>
<td>2</td>
<td>高抬腿</td>
</tr>
<tr class="even">
<td></td>
<td>situp</td>
<td>3</td>
<td>仰卧起坐</td>
</tr>
<tr class="odd">
<td></td>
<td>pressup</td>
<td>4</td>
<td>俯卧撑</td>
</tr>
<tr class="even">
<td></td>
<td>plank</td>
<td>5</td>
<td>平板支撑</td>
</tr>
<tr class="odd">
<td></td>
<td>jumpjack</td>
<td>6</td>
<td>开合跳</td>
</tr>
</tbody>
</table>

**SkeletonRecognizedSeviceResponse：骨骼点识别服务反馈消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>SkeletonRecognizedSeviceResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>result</td>
<td>int</td>
<td>服务反馈状态，0:成功，1:失败</td>
</tr>
</tbody>
</table>

**SkeletonRecognizedMessageResponse：骨骼点识别消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>SkeletonRecognizedMessageResponse</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合</p>
<p>法</p>
<p>字</p>
<p>段</p>
<p>约</p>
<p>束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>algo_switch</td>
<td>int</td>
<td>算法开关，0:打开，1:关闭</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td>sport_type</td>
<td>int</td>
<td>算法识别类型，参见SkeletonType</td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td>counts</td>
<td>int</td>
<td>运动计数，从1开始</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td>duration</td>
<td>int</td>
<td>运动时长(平板支撑计时长度)</td>
</tr>
</tbody>
</table>

**MsgTrainingWords：训练词信息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>MsgTrainingWords</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>trigger</td>
<td>string</td>
<td>关键词</td>
</tr>
<tr class="odd">
<td></td>
<td>type</td>
<td>string</td>
<td>类型</td>
</tr>
<tr class="even">
<td></td>
<td>value</td>
<td>string</td>
<td>值</td>
</tr>
</tbody>
</table>

**TrainingWordsRecognizedSeviceResponse：训练词服务反馈**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>TrainingWordsRecognizedSeviceResponse</td>
<td></td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td></td>
<td>类型</td>
<td>含义</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td></td>
<td><a>State</a></td>
<td>状态</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td>training_set</td>
<td>list&lt;<a>MsgTrainingWords</a>&gt;</td>
<td>人脸识别列表</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>可用操作</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>接口</td>
<td>说明</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>empty();</td>
<td>详情参见<a>列表</a></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>size();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>max_size();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>capacity();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>clear();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>append();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>pop();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>at();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>front();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>back();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td>dictionary</td>
<td></td>
<td>dictionary&lt;string, <a>MsgTrainingWords</a> &gt;</td>
<td>人脸识别字典</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>可用操作</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>接口</td>
<td>说明</td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>empty();</td>
<td>详情参见<a>字典</a></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>size();</td>
<td></td>
</tr>
<tr class="even">
<td></td>
<td></td>
<td></td>
<td></td>
<td>max_size();</td>
<td></td>
</tr>
<tr class="odd">
<td></td>
<td></td>
<td></td>
<td></td>
<td>clear();</td>
<td></td>
</tr>
</tbody>
</table>

**TrainingWordsRecognizedMessageResponse：训练词识别消息**

<table>
<tbody>
<tr class="odd">
<td>类别</td>
<td>结构</td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td>类型</td>
<td>TrainingWordsRecognizedMessageResponse</td>
<td></td>
<td></td>
</tr>
<tr class="odd">
<td><p>合法</p>
<p>字段</p>
<p>约束</p></td>
<td>字段</td>
<td>类型</td>
<td>含义</td>
</tr>
<tr class="even">
<td></td>
<td>state</td>
<td><a>State</a></td>
<td>状态</td>
</tr>
<tr class="odd">
<td></td>
<td>response</td>
<td><a>MsgTrainingWords</a></td>
<td>训练词</td>
</tr>
</tbody>
</table>

**4.3 铁蛋能力集接口约束表**

|                                                                           |    |                                     |
| ------------------------------------------------------------------------- | -- | ----------------------------------- |
| 图例                                                                        | 含义 | 说明                                  |
| [🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) | 字段 | 标识该接口为变量，可以直接取值。                    |
| [🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) | 函数 | 标识该接口为函数，主体功能在函数退出后即结束，根据返回值判断执行情况。 |
| 🟡                                                                         | 句柄 | 标识该接口为对象，无法直接调用，只能调用其下的字段和函数。       |

**4.4 铁蛋能力集接口约束**

**4.4.01 🟡cyberdog（铁蛋）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.state</strong>: cyberdog模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.set_log</strong>: 设置cyberdog模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) shutdown（退出）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.shutdown()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.shutdown</strong>: 退出cyberdog。</p>
<p>参数：无</p>
<p>返回值：无</p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.shutdown()</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.02 🟡network（网络模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.network.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.network.state</strong>: cyberdog下网络模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.network.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.network.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.network.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.network.data</strong>: cyberdog下网络模块状态获取接口名称。</p>
<p>类型：<a>ConnectorStatus</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.network.data.is_connected:</strong><br />
<strong>print('当前机器人已连接到', cyberdog.network.data.ssid, 'WiFi网络')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.network.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.network.set_log</strong>: 设置cyberdog下network模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.network.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下network模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.03 🟡follow（跟随模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.follow.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.follow.state</strong>: cyberdog下跟随模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.follow.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.follow.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.follow.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.follow.set_log</strong>: 设置cyberdog下follow模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.follow.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下follow模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) add\_personnel（添加人员）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.follow.add_personnel(</strong><br />
<strong>string preset_name)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.follow.add_personnel</strong>: 添加跟随人员信息。</p>
<p>参数：</p>
<p><strong>preset_name</strong>: 人员名称，字符串类型；</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.follow.add_personnel('张三').code == StateCode.success:</strong><br />
<strong>print('添加张三到跟随模块成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) delete\_personnel（删除人员）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.follow.delete_personnel(</strong><br />
<strong>string preset_name)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.follow.add_personnel</strong>: 删除跟随人员信息。</p>
<p>参数：</p>
<p><strong>preset_name</strong>: 人员名称，字符串类型；</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.follow.delete_personnel('张三').code == StateCode.success:</strong><br />
<strong>print('从跟随模块删除张三成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) follow\_personnel（跟随人员）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.follow.follow_personnel(</strong><br />
<strong>string preset_name,</strong><br />
<strong>double intimacy)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.follow.follow_personnel</strong>: 添加跟随人员信息。</p>
<p>参数：</p>
<p><strong>preset_name</strong>: 人员名称，字符串类型；</p>
<p><strong>intimacy</strong>: 跟随间距，浮点数类型；</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.follow.follow_personnel('张三', 1.0).code == StateCode.success:</strong><br />
<strong>print('开启跟随张三成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) cancel\_follow（取消跟随）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.follow.cancel_follow(</strong><br />
<strong>string preset_name)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.follow.cancel_follow</strong>: 删除跟随人员信息。</p>
<p>参数：</p>
<p><strong>preset_name</strong>: 人员名称，字符串类型；</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.follow.cancel_follow('张三').code == StateCode.success:</strong><br />
<strong>print('取消跟随张三成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.04 🟡motion（运动模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.state</strong>: cyberdog下运动模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.motion.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟣 params（参数）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.params</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.params</strong>: cyberdog下运动模块默认参数获取接口名称。</p>
<p>类型：<a>MotionParams</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.motion.params)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.set_log</strong>: 设置cyberdog下motion模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下motion模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) emergency\_stop（急停）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.emergency_stop()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.emergency_stop</strong>: 使得机器人急停。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.emergency_stop().state.code == StateCode.success:</strong><br />
<strong>print('使机器人急停成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_down（趴下）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.get_down()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.get_down</strong>: 使得机器人趴下。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.get_down().state.code == StateCode.success:</strong><br />
<strong>print('使机器人趴下成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) resume\_standing（站立）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.resume_standing()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.resume_standing</strong>: 使得机器人恢复站立。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.resume_standing().state.code == StateCode.success:</strong><br />
<strong>print('使机器人站立成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) back\_flip（后空翻）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.back_flip()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.back_flip</strong>: 使得机器人后空翻。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.back_flip().state.code == StateCode.success:</strong><br />
<strong>print('使机器人后空翻成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) front\_flip（前空翻）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.front_flip()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.front_flip</strong>: 使得机器人前空翻。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.front_flip().state.code == StateCode.success:</strong><br />
<strong>print('使机器人前空翻成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) bow（作揖）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.bow()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.bow</strong>: 使得机器人作揖。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.bow().state.code == StateCode.success:</strong><br />
<strong>print('使机器人作揖成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) roll\_left（向左侧躺后恢复）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.roll_left()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.roll_left</strong>: 使得机器人向左侧躺后恢复。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.roll_left().state.code == StateCode.success:</strong><br />
<strong>print('使机器人向左侧躺后恢复成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) walk\_the\_dog（遛狗）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.walk_the_dog(</strong><br />
<strong>double front_leg_lift</strong>,<br />
<strong>double</strong> <strong>back_leg_lift)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.walk_the_dog</strong>: 使得机器人进入遛狗模式。</p>
<p>参数：</p>
<p><strong>front_leg_lift</strong>: 前脚抬腿高度，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>back_leg_lift</strong>: 前脚抬腿高度，单位秒（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.walk_the_dog(0.03，0.03).state.code == StateCode.success:</strong><br />
<strong>print('使机器人进入遛狗模式成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump\_stair（跳上台阶）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.jump_stair()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump_stair</strong>: 使得机器人跳上台阶。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump_stair().state.code == StateCode.success:</strong><br />
<strong>print('使机器人跳上台阶成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) right\_somersault（右侧空翻）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.right_somersault()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.right_somersault</strong>: 使得机器人右侧空翻。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.right_somersault().state.code == StateCode.success:</strong><br />
<strong>print('使机器人右侧空翻成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) left\_somersault（左侧空翻）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.left_somersault()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.left_somersault</strong>: 使得机器人左侧空翻。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.left_somersault().state.code == StateCode.success:</strong><br />
<strong>print('使机器人左侧空翻成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) run\_and\_jump\_front\_flip（跑跳前空翻）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.run_and_jump_front_flip()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.run_and_jump_front_flip</strong>: 使得机器人跑跳前空翻。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.run_and_jump_front_flip().state.code == StateCode.success:</strong><br />
<strong>print('使机器人跑跳前空翻成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump3d\_left90deg（3D跳:左转90度）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.jump3d_left90deg()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump3d_left90deg</strong>: 使得机器人3D跳:左转90度。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump3d_left90deg().state.code == StateCode.success:</strong><br />
<strong>print('使机器人3D跳:左转90度成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump3d\_right90deg（3D跳:右转90度）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>cyberdog.motion.jump3d_right90deg()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump3d_right90deg</strong>: 使得机器人3D跳:右转90度。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump3d_right90deg().state.code == StateCode.success:</strong><br />
<strong>print('使机器人3D跳:右转90度成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump3d\_forward60cm（3D跳:前跳60cm）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>cyberdog.motion.jump3d_forward60cm()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump3d_forward60cm</strong>: 使得机器人3D跳:前跳60cm。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump3d_forward60cm().state.code == StateCode.success:</strong><br />
<strong>print('使机器人3D跳:前跳60cm成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump3d\_forward30cm（3D跳:前跳30cm）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>cyberdog.motion.jump3d_forward30cm()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump3d_forward30cm</strong>: 使得机器人3D跳:前跳30cm。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump3d_forward30cm().state.code == StateCode.success:</strong><br />
<strong>print('使机器人3D跳:前跳30cm成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump3d\_left20cm（3D跳:左跳20cm）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>cyberdog.motion.jump3d_left20cm()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump3d_left20cm</strong>: 使得机器人3D跳:左跳20cm。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump3d_left20cm().state.code == StateCode.success:</strong><br />
<strong>print('使机器人3D跳:左跳20cm成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump3d\_right20cm（3D跳:右跳20cm）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>cyberdog.motion.jump3d_right20cm()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump3d_right20cm</strong>: 使得机器人3D跳:右跳20cm。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump3d_right20cm().state.code == StateCode.success:</strong><br />
<strong>print('使机器人3D跳:右跳20cm成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump3d\_up30cm（3D跳:向上30cm）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td>Apache<br />
<strong>cyberdog.motion.jump3d_up30cm()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump3d_up30cm</strong>: 使得机器人3D跳:向上30cm。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump3d_up30cm().state.code == StateCode.success:</strong><br />
<strong>print('使机器人3D跳:向上3D跳:30cm成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump3d\_down\_stair（3D跳:跳下台阶）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.jump3d_down_stair()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump3d_down_stair</strong>: 使得机器人3D跳:跳下台阶。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump3d_down_stair().state.code == StateCode.success:</strong><br />
<strong>print('使机器人3D跳:跳下台阶成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) roll\_right（向右侧躺后恢复）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.roll_right()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.roll_right</strong>: 使得机器人向右侧躺后恢复。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.roll_right().state.code == StateCode.success:</strong><br />
<strong>print('使机器人向右侧躺后恢复成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) dance\_collection（舞蹈集合）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.dance_collection()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.dance_collection</strong>: 使得机器人舞蹈集合。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.dance_collection().state.code == StateCode.success:</strong><br />
<strong>print('使机器人执行舞蹈集合成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) hold\_right\_hand（握左手）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.hold_left_hand()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.hold_left_hand</strong>: 使得机器人握左手。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.hold_left_hand().state.code == StateCode.success:</strong><br />
<strong>print('使机器人握左手成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) hold\_right\_hand（握右手）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.hold_right_hand()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.hold_right_hand</strong>: 使得机器人握右手。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.hold_right_hand().state.code == StateCode.success:</strong><br />
<strong>print('使机器人握右手成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) sit\_down（坐下）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.sit_down()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.sit_down</strong>: 使得机器人坐下。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.sit_down().state.code == StateCode.success:</strong><br />
<strong>print('使机器人坐下成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) butt\_circle（屁股画圆）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.butt_circle()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.butt_circle</strong>: 使得机器人屁股画圆。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.butt_circle().state.code == StateCode.success:</strong><br />
<strong>print('使机器人屁股画圆成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) head\_circle（头画圆）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.head_circle()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.head_circle</strong>: 使得机器人头画圆。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.head_circle().state.code == StateCode.success:</strong><br />
<strong>print('使机器人头画圆成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) stretch\_the\_body（伸展身体）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.stretch_the_body()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.stretch_the_body</strong>: 使得机器人伸展身体。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.stretch_the_body().state.code == StateCode.success:</strong><br />
<strong>print('使机器人伸展身体成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) shake\_ass\_left（向左摇晃屁股）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.shake_ass_left()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.shake_ass_left</strong>: 使得机器人向左摇晃屁股。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p>
<p>备注：必须在坐下之后进行。</p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.shake_ass_left().state.code == StateCode.success:</strong><br />
<strong>print('使机器人向左摇晃屁股成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) shake\_ass\_right（向右摇晃屁股）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.shake_ass_right()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.shake_ass_right</strong>: 使得机器人向右摇晃屁股。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p>
<p>备注：必须在坐下之后进行。</p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.shake_ass_right().state.code == StateCode.success:</strong><br />
<strong>print('使机器人向右摇晃屁股成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) shake\_ass\_from\_side\_to\_side（左右摇晃屁股）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.shake_ass_from_side_to_side()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.shake_ass_from_side_to_side</strong>: 使得机器人左右摇晃屁股。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p>
<p>备注：必须在坐下之后进行。</p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.shake_ass_from_side_to_side().state.code == StateCode.success:</strong><br />
<strong>print('使机器人左右摇晃屁股成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) ballet（芭蕾舞）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.ballet()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.ballet</strong>: 使得机器人芭蕾舞。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.ballet().state.code == StateCode.success:</strong><br />
<strong>print('使机器人芭蕾舞成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) space\_walk（太空步）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.space_walk()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.space_walk</strong>: 使得机器人太空步。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.space_walk().state.code == StateCode.success:</strong><br />
<strong>print('使机器人太空步成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) front\_leg\_jumping（前腿开合跳）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.front_leg_jumping()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.front_leg_jumping</strong>: 使得机器人前腿开合跳。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.front_leg_jumping().state.code == StateCode.success:</strong><br />
<strong>print('使机器人前腿开合跳成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) hind\_leg\_jumping（后腿开合跳）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.hind_leg_jumping()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.hind_leg_jumping</strong>: 使得机器人后腿开合跳。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.hind_leg_jumping().state.code == StateCode.success:</strong><br />
<strong>print('使机器人后腿开合跳成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) lift\_the\_left\_leg\_and\_nod（左腿抬起并点头）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.lift_the_left_leg_and_nod()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.lift_the_left_leg_and_nod</strong>: 使得机器人左腿抬起并点头。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.lift_the_left_leg_and_nod().state.code == StateCode.success:</strong><br />
<strong>print('使机器人左腿抬起并点头成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) lift\_the\_right\_leg\_and\_nod（右腿抬起并点头）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.lift_the_right_leg_and_nod()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.lift_the_right_leg_and_nod</strong>: 使得机器人右腿抬起并点头。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.lift_the_right_leg_and_nod().state.code == StateCode.success:</strong><br />
<strong>print('使机器人右腿抬起并点头成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) left\_front\_right\_back\_legs\_apart（左前右后岔开腿）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.left_front_right_back_legs_apart()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.left_front_right_back_legs_apart</strong>: 使得机器人左前右后岔开腿。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.left_front_right_back_legs_apart().state.code == StateCode.success:</strong><br />
<strong>print('使机器人左前右后岔开腿成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) right\_front\_left\_back\_legs\_apart（右前左后岔开腿）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.right_front_left_back_legs_apart()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.right_front_left_back_legs_apart</strong>: 使得机器人右前左后岔开腿。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.right_front_left_back_legs_apart().state.code == StateCode.success:</strong><br />
<strong>print('使机器人右前左后岔开腿成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) walk\_nodding（走路点头）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.walk_nodding()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.walk_nodding</strong>: 使得机器人走路点头。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.walk_nodding().state.code == StateCode.success:</strong><br />
<strong>print('使机器人走路点头成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) walking\_with\_divergence\_and\_adduction\_alternately（岔开内收交替走路）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.walking_with_divergence_and_adduction_alternately()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.walking_with_divergence_and_adduction_alternately</strong>: 使得机器人岔开内收交替走路。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.walking_with_divergence_and_adduction_alternately().state.code == StateCode.success:</strong><br />
<strong>print('使机器人岔开内收交替走路成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) nodding\_in\_place（原地踏步点头）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.nodding_in_place()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.nodding_in_place</strong>: 使得机器人原地踏步点头。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.nodding_in_place().state.code == StateCode.success:</strong><br />
<strong>print('使机器人原地踏步点头成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) front\_legs\_jump\_back\_and\_forth（前腿前后跳）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.front_legs_jump_back_and_forth()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.front_legs_jump_back_and_forth</strong>: 使得机器人前腿前后跳。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.front_legs_jump_back_and_forth().state.code == StateCode.success:</strong><br />
<strong>print('使机器人前腿前后跳成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) hind\_legs\_jump\_back\_and\_forth（后腿前后跳）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.hind_legs_jump_back_and_forth()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.hind_legs_jump_back_and_forth</strong>: 使得机器人后腿前后跳。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.hind_legs_jump_back_and_forth().state.code == StateCode.success:</strong><br />
<strong>print('使机器人后腿前后跳成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) alternately\_front\_leg\_lift（前腿交替抬起）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.alternately_front_leg_lift()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.alternately_front_leg_lift</strong>: 使得机器人前腿交替抬起。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.alternately_front_leg_lift().state.code == StateCode.success:</strong><br />
<strong>print('使机器人前腿交替抬起成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) alternately\_hind\_leg\_lift（后腿交替抬起）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.alternately_hind_leg_lift()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.alternately_hind_leg_lift</strong>: 使得机器人后腿交替抬起。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.alternately_hind_leg_lift().state.code == StateCode.success:</strong><br />
<strong>print('使机器人后腿交替抬起成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump\_collection（跳跃合集）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.jump_collection()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump_collection</strong>: 使得机器人跳跃合集。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump_collection().state.code == StateCode.success:</strong><br />
<strong>print('使机器人跳跃合集成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) stretching\_left\_and\_right（左右伸腿踏步）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.stretching_left_and_right()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.stretching_left_and_right</strong>: 使得机器人左右伸腿踏步。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.stretching_left_and_right().state.code == StateCode.success:</strong><br />
<strong>print('使机器人左右伸腿踏步成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump\_forward\_and\_backward（前后摆腿跳跃）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.jump_forward_and_backward()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump_forward_and_backward</strong>: 使得机器人前后摆腿跳跃。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump_forward_and_backward().state.code == StateCode.success:</strong><br />
<strong>print('使机器人前后摆腿跳跃成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) step\_left\_and\_right（左右摆腿踏步）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.step_left_and_right()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.step_left_and_right</strong>: 使得机器人左右摆腿踏步。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.step_left_and_right().state.code == StateCode.success:</strong><br />
<strong>print('使机器人左右摆腿踏步成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) right\_leg\_back\_and\_forth\_stepping（右腿前后踏步）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.right_leg_back_and_forth_stepping()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.right_leg_back_and_forth_stepping</strong>: 使得机器人右腿前后踏步。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.right_leg_back_and_forth_stepping().state.code == StateCode.success:</strong><br />
<strong>print('使机器人右腿前后踏步成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) left\_leg\_back\_and\_forth\_stepping（左腿前后踏步**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.left_leg_back_and_forth_stepping()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.left_leg_back_and_forth_stepping</strong>: 使得机器人左腿前后踏步。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.left_leg_back_and_forth_stepping().state.code == StateCode.success:</strong><br />
<strong>print('使机器人左腿前后踏步成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) squat\_down\_on\_all\_fours（四足蹲起）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.squat_down_on_all_fours()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.squat_down_on_all_fours</strong>: 使得机器人四足蹲起。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.squat_down_on_all_fours().state.code == StateCode.success:</strong><br />
<strong>print('使机器人四足蹲起成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) push\_ups（俯卧撑）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.push_ups()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.push_ups</strong>: 使得做机器人俯卧撑。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.push_ups().state.code == StateCode.success:</strong><br />
<strong>print('使机器人俯卧撑成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) bow\_to\_each\_other（作揖比心）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.bow_to_each_other()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.bow_to_each_other</strong>: 使得机器人作揖比心。</p>
<p>参数：无</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.bow_to_each_other().state.code == StateCode.success:</strong><br />
<strong>print('使机器人作揖比心成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) absolute\_attitude（绝对姿态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.absolute_attitude(</strong><br />
<strong>double centroid_z,</strong><br />
<strong>double roll，</strong><br />
<strong>double pitch,</strong><br />
<strong>double yaw，</strong><br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.absolute_attitude</strong>: 使得机器人绝对力控姿态。</p>
<p>参数：</p>
<p><strong>centroid_z:</strong>质心高度，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>roll</strong>: 机身翻滚，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>pitch</strong>: 机身俯仰，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>yaw</strong>: 机身偏航，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration:</strong>转变姿态期望耗时，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值0</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.absolute_attitude(0.2,5,5,5,1).state.code == StateCode.success:</strong><br />
<strong>print('控制机器人绝对姿态成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) relatively\_force\_control\_attitude（相对力控姿态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.relatively_force_control_attitude(</strong><br />
<strong>double centroid_x,</strong><br />
<strong>double centroid_y,</strong><br />
<strong>double centroid_z,</strong><br />
<strong>double roll，</strong><br />
<strong>double pitch,</strong><br />
<strong>double yaw，</strong><br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.relatively_force_control_attitude</strong>: 使得机器人相对当前姿态进行力控姿态动作。</p>
<p>参数：</p>
<p><strong>centroid_x:</strong>质心x轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>centroid_y:</strong>质心y轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>centroid_z:</strong>质心z轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>roll</strong>: 机身翻滚，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>pitch</strong>: 机身俯仰，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>yaw</strong>: 机身偏航，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration:</strong>转变姿态期望耗时，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值1</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.relatively_force_control_attitude(0.2,0.2,0.2,5,5,5,1).state.code == StateCode.success:</strong><br />
<strong>print('控制机器人相对力控姿态成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) transition\_standing（过渡站立）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.transition_standing(</strong><br />
<strong>double centroid_z,</strong><br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.transition_standing</strong>: 使得机器人过渡站立。</p>
<p>参数：</p>
<p><strong>centroid_z:</strong>质心高度，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration:</strong>转变姿态期望耗时，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值1</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.transition_standing(0.2,1).state.code == StateCode.success:</strong><br />
<strong>print('控制机器人过渡站立成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) relatively\_position\_control\_attitude（相对位控姿态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.relatively_position_control_attitude(</strong><br />
<strong>double centroid_x,</strong><br />
<strong>double centroid_y,</strong><br />
<strong>double centroid_z,</strong><br />
<strong>double roll，</strong><br />
<strong>double pitch,</strong><br />
<strong>double yaw，</strong><br />
<strong>double fulcrum_x,</strong><br />
<strong>double fulcrum_y,</strong><br />
<strong>double fulcrum_z,</strong><br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.relatively_position_control_attitude</strong>: 使得机器人相对力控姿态。</p>
<p>参数：</p>
<p><strong>centroid_x</strong>:质心x轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>centroid_y</strong>:质心y轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>centroid_z</strong>:质心z轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>roll</strong>: 机身翻滚，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>pitch</strong>: 机身俯仰，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>yaw</strong>: 机身偏航，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>fulcrum_x</strong>:支点x轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>fulcrum_y</strong>:支点y轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>fulcrum_z</strong>:支点z轴坐标，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration:</strong>转变姿态期望耗时，类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值1</p>
<p>返回值：<a>MotionResultServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.relatively_position_control_attitude(0.2,0.2,0.2,5,5,5,0,0,0,1).state.code == StateCode.success:</strong><br />
<strong>print('控制机器人相对位控姿态成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) jump\_back\_and\_forth（前后跳）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.jump_back_and_forth(</strong><br />
<strong>double x_jump_velocity,</strong><br />
<strong>double y_jump_velocity,</strong><br />
<strong>double z_jump_velocity,</strong><br />
<strong>double front_leg_lift,</strong><br />
<strong>double back_leg_lift，</strong><br />
<strong>double distance,</strong><br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.jump_back_and_forth</strong>: 使得机器人前后跳。</p>
<p>参数：</p>
<p><strong>x_jump_velocity</strong>: 纵向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>y_jump_velocity</strong>: 横向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>z_jump_velocity</strong>: 角速度，单位度每秒（<a href="https://zh.wikipedia.org/wiki/°">°</a>/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>front_leg_lift</strong>: 前脚抬腿高度，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>back_leg_lift</strong>: 前脚抬腿高度，单位秒（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>distance</strong>: 期望距离，路程，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>备注：</p>
<p>distance和duration互斥，同时出现时以第一个非零值进行约束。</p>
<p>返回值：<a>MotionServoCmdResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump_back_and_forth(0.03，0.03).state.code == StateCode.success:</strong><br />
<strong>print('使机器人前后跳成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) small\_jump\_walking（小跳行走）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.small_jump_walking(</strong><br />
<strong>double x_jump_velocity</strong>,<br />
<strong>double y_jump_velocity</strong>,<br />
<strong>double z_jump_velocity</strong>,<br />
<strong>double front_leg_lift</strong>,<br />
<strong>double back_leg_lift</strong>，<br />
<strong>double distance</strong>,<br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.small_jump_walking</strong>: 使得机器人小跳行走。</p>
<p>参数</p>
<p><strong>x_jump_velocity</strong>: 纵向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>y_jump_velocity</strong>: 横向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>z_jump_velocity</strong>: 角速度，单位度每秒（<a href="https://zh.wikipedia.org/wiki/°">°</a>/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>front_leg_lift</strong>: 前脚抬腿高度，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>back_leg_lift</strong>: 前脚抬腿高度，单位秒（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>distance</strong>: 期望距离，路程，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>备注：</p>
<p><strong>distance</strong>和<strong>duration</strong>互斥，同时出现时以第一个非零值进行约束。</p>
<p>返回值：<a>MotionServoCmdResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.jump_back_and_forth(0.2，0, 0.0, 0.02, 0.02，0，1).state.code == StateCode.success:</strong><br />
<strong>print('使机器人小跳行走成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) trot\_walking（慢速行走）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.trot_walking(</strong><br />
<strong>double x_velocity</strong>,<br />
<strong>double y_velocity</strong>,<br />
<strong>double z_velocity</strong>,<br />
<strong>double front_leg_lift</strong>,<br />
<strong>double back_leg_lift</strong>，<br />
<strong>double distance</strong>,<br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.trot_walking</strong>: 使得机器人慢速行走。</p>
<p>参数</p>
<p><strong>x_velocity</strong>: 纵向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>y_velocity</strong>: 横向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>z_velocity</strong>: 角速度，单位度每秒（<a href="https://zh.wikipedia.org/wiki/°">°</a>/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>front_leg_lift</strong>: 前脚抬腿高度，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>back_leg_lift</strong>: 前脚抬腿高度，单位秒（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>distance</strong>: 期望距离，路程，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>备注：</p>
<p><strong>distance</strong>和<strong>duration</strong>互斥，同时出现时以第一个非零值进行约束。</p>
<p>返回值：<a>MotionServoCmdResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.trot_walking(0.2，0.3, 0.0, 0.02, 0.02).state.code == StateCode.success:</strong><br />
<strong>print('使机器人慢速行走成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) automatic\_frequency\_conversion\_walking（自动变频行走）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.automatic_frequency_conversion_walking(</strong><br />
<strong>double x_velocity</strong>,<br />
<strong>double y_velocity</strong>,<br />
<strong>double z_velocity</strong>,<br />
<strong>double front_leg_lift</strong>,<br />
<strong>double back_leg_lift</strong>，<br />
<strong>double distance</strong>,<br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.automatic_frequency_conversion_walking</strong>: 使得机器人自动变频行走。</p>
<p>参数</p>
<p><strong>x_velocity</strong>: 纵向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>y_velocity</strong>: 横向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>z_velocity</strong>: 角速度，单位度每秒（<a href="https://zh.wikipedia.org/wiki/°">°</a>/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>front_leg_lift</strong>: 前脚抬腿高度，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。0.03</p>
<p><strong>back_leg_lift</strong>: 前脚抬腿高度，单位秒（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。0.03</p>
<p><strong>distance</strong>: 期望距离，路程，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>备注：</p>
<p><strong>distance</strong>和<strong>duration</strong>互斥，同时出现时以第一个非零值进行约束。</p>
<p>返回值：<a>MotionServoCmdResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.automatic_frequency_conversion_walking(0.2，0.3, 0.0, 0.02, 0.02).state.code == StateCode.success:</strong><br />
<strong>print('使机器人自动变频行走成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) run\_fast\_walking（快跑行走）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.run_fast_walking(</strong><br />
<strong>double x_velocity</strong>,<br />
<strong>double y_velocity</strong>,<br />
<strong>double z_velocity</strong>,<br />
<strong>double front_leg_lift</strong>,<br />
<strong>double back_leg_lift</strong>，<br />
<strong>double distance</strong>,<br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.run_fast_walking</strong>: 使得机器人快跑。</p>
<p>参数</p>
<p><strong>x_velocity</strong>: 纵向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>y_velocity</strong>: 横向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>z_velocity</strong>: 角速度，单位度每秒（<a href="https://zh.wikipedia.org/wiki/°">°</a>/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>front_leg_lift</strong>: 前脚抬腿高度，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>back_leg_lift</strong>: 前脚抬腿高度，单位秒（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>distance</strong>: 期望距离，路程，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>备注：</p>
<p><strong>distance</strong>和<strong>duration</strong>互斥，同时出现时以第一个非零值进行约束。</p>
<p>返回值：<a>MotionServoCmdResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.run_fast_walking(0.2，0.3, 0.0, 0.02, 0.02).state.code == StateCode.success:</strong><br />
<strong>print('使机器人快跑成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) turn（转向）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.turn(</strong><br />
<strong>double</strong> <strong>angle</strong>,<br />
<strong>double</strong> <strong>duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.turn</strong>: 控制机器人转向。</p>
<p>参数</p>
<p><strong>angle</strong>: 角度，单位度（<a href="https://zh.wikipedia.org/wiki/°">°</a>），类型为浮点型；</p>
<p>以当前姿态为0度，顺时针为负</p>
<p>有效范围：[-360, 360)。</p>
<p>默认：0</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值：1</p>
<p>返回值：<a>MotionServoCmdResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.turn(10).state.code == StateCode.success:</strong><br />
<strong>print('使机器人向左转向10度成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) go\_straight（直行）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.go_straight(</strong><br />
<strong>double x_velocity</strong>，<br />
<strong>double</strong> <strong>distance</strong>,<br />
<strong>double</strong> <strong>duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog. motion.go_straight</strong>: 控制机器人直行。</p>
<p>参数</p>
<p><strong>x_velocity</strong>: 纵向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>distance</strong>: 期望距离，路程，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值：1</p>
<p>备注：</p>
<p><strong>distance</strong>和<strong>duration</strong>互斥，同时出现时以第一个非零值进行约束。</p>
<p>返回值：<a>MotionServoCmdResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.go_straight(0.3, 0, 10).state.code == StateCode.success:</strong><br />
<strong>print('使机器人以0.3m/s的速度向前10s成功')</strong><br />
<strong>if cyberdog.motion.go_straight(-0.3, 0, 10).state.code == StateCode.success:</strong><br />
<strong>print('使机器人以0.3m/s的速度向后10s成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) lateral\_movement（横移）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.lateral_movement(</strong><br />
<strong>double y_velocity</strong>，<br />
<strong>double</strong> <strong>distance</strong>,<br />
<strong>double</strong> <strong>duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.lateral_movement</strong>: 横移接口名称，控制机器人横移。</p>
<p>参数</p>
<p><strong>y_velocity</strong>: 横向线速度，单位米每秒（m/s），类型为浮点型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>distance</strong>: 期望距离，路程，单位米（m），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值：1</p>
<p>备注：</p>
<p><strong>distance</strong>和<strong>duration</strong>互斥，同时出现时以第一个非零值进行约束。</p>
<p>返回值：<a>MotionServoCmdResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.motion.lateral_movement(0.3, 0, 10).state.code == StateCode.success:</strong><br />
<strong>print('使机器人以0.3m/s的速度向左10s成功')</strong><br />
<strong>if cyberdog.motion.lateral_movement(-0.3, 0, 10).state.code == StateCode.success:</strong><br />
<strong>print('使机器人以0.3m/s的速度向右10s成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) run\_sequence（运行序列）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.motion.run_sequence(</strong><br />
<strong>MotionSequence sequence</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.motion.run_sequence</strong>: 运行序列接口名称，使机器人运行当前序列。</p>
<p>参数</p>
<p><strong>sequence</strong>: 序列，类型为<a>MotionSequence</a>。</p>
<p>返回值：<a>MotionSequenceServiceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td>Python<br />
<br />
sequ = MotionSequence()<br />
sequ.name = 'test_sequ'<br />
sequ.describe = '测试序列'<br />
<br />
gait_meta = MotionSequenceGait()<br />
<br />
gait_meta.right_forefoot = 1<br />
gait_meta.left_forefoot = 1<br />
gait_meta.right_hindfoot = 1<br />
gait_meta.left_hindfoot = 1<br />
gait_meta.duration = 1<br />
sequ.gait_list.push_back(gait_meta)<br />
<br />
gait_meta.right_forefoot = 0<br />
gait_meta.left_forefoot = 1<br />
gait_meta.right_hindfoot = 1<br />
gait_meta.left_hindfoot = 0<br />
gait_meta.duration = 1<br />
sequ.gait_list.push_back(gait_meta)<br />
<br />
gait_meta.right_forefoot = 1<br />
gait_meta.left_forefoot = 1<br />
gait_meta.right_hindfoot = 1<br />
gait_meta.left_hindfoot = 1<br />
gait_meta.duration = 1<br />
sequ.gait_list.push_back(gait_meta)<br />
<br />
pace_meta = MotionSequencePace()<br />
<br />
pace_meta.twist.linear.x = 0.000000<br />
pace_meta.twist.linear.y = 0.000000<br />
pace_meta.twist.linear.z = 0.000000<br />
pace_meta.centroid.position.x = 0.000000<br />
pace_meta.centroid.position.y = 0.000000<br />
pace_meta.centroid.position.z = 0.020000<br />
pace_meta.centroid.orientation.x = 0.000000<br />
pace_meta.centroid.orientation.y = 0.000000<br />
pace_meta.centroid.orientation.z = 0.000000<br />
pace_meta.weight.linear.x = 50.000000<br />
pace_meta.weight.linear.y = 50.000000<br />
pace_meta.weight.linear.z = 5.000000<br />
pace_meta.weight.angular.x = 10.000000<br />
pace_meta.weight.angular.y = 10.000000<br />
pace_meta.weight.angular.z = 10.000000<br />
pace_meta.right_forefoot.x = 0.030000<br />
pace_meta.right_forefoot.y = 0.050000<br />
pace_meta.left_forefoot.x = -0.030000<br />
pace_meta.left_forefoot.y = -0.050000<br />
pace_meta.right_hindfoot.x = 0.030000<br />
pace_meta.right_hindfoot.y = 0.050000<br />
pace_meta.left_hindfoot.x = -0.030000<br />
pace_meta.left_hindfoot.y = -0.050000<br />
pace_meta.friction_coefficient = 0.400000<br />
pace_meta.right_forefoot.w = 0.030000<br />
pace_meta.left_forefoot.w = 0.030000<br />
pace_meta.right_hindfoot.w = 0.030000<br />
pace_meta.left_hindfoot.w = 0.030000<br />
pace_meta.landing_gain = 1.500000<br />
pace_meta.use_mpc_track = 0<br />
pace_meta.duration = 300<br />
sequ.pace_list.push_back(pace_meta)<br />
<br />
pace_meta.twist.linear.x = 0.000000<br />
pace_meta.twist.linear.y = 0.000000<br />
pace_meta.twist.linear.z = 0.000000<br />
pace_meta.centroid.position.x = 0.000000<br />
pace_meta.centroid.position.y = 0.000000<br />
pace_meta.centroid.position.z = 0.000000<br />
pace_meta.centroid.orientation.x = 0.000000<br />
pace_meta.centroid.orientation.y = 0.000000<br />
pace_meta.centroid.orientation.z = 0.000000<br />
pace_meta.weight.linear.x = 50.000000<br />
pace_meta.weight.linear.y = 50.000000<br />
pace_meta.weight.linear.z = 5.000000<br />
pace_meta.weight.angular.x = 10.000000<br />
pace_meta.weight.angular.y = 10.000000<br />
pace_meta.weight.angular.z = 10.000000<br />
pace_meta.right_forefoot.x = 0.000000<br />
pace_meta.right_forefoot.y = 0.000000<br />
pace_meta.left_forefoot.x = 0.000000<br />
pace_meta.left_forefoot.y = 0.000000<br />
pace_meta.right_hindfoot.x = 0.000000<br />
pace_meta.right_hindfoot.y = 0.000000<br />
pace_meta.left_hindfoot.x = 0.000000<br />
pace_meta.left_hindfoot.y = 0.000000<br />
pace_meta.friction_coefficient = 0.400000<br />
pace_meta.right_forefoot.w = 0.030000<br />
pace_meta.left_forefoot.w = 0.030000<br />
pace_meta.right_hindfoot.w = 0.030000<br />
pace_meta.left_hindfoot.w = 0.030000<br />
pace_meta.landing_gain = 1.500000<br />
pace_meta.use_mpc_track = 0<br />
pace_meta.duration = 300<br />
sequ.pace_list.push_back(pace_meta)<br />
<br />
cyberdog.motion.run_sequence(sequ)</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.05 🟡navigation（导航模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.state</strong>: cyberdog下导航模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.set_log</strong>: 设置cyberdog下navigation模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下navigation模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_preset（获取预置点）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.get_preset(</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.get_preset</strong>: 获取预置点。</p>
<p>参数</p>
<p>返回值：<a>MapPresetSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation. get_preset().state.code == StateCode.success:</strong><br />
<strong>print('添加厨房预置点成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) to\_preset（导航到预置点）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.to_preset(</strong><br />
<strong>string preset_name,</strong><br />
<strong>bool assisted_relocation,</strong><br />
<strong>bool interact,</strong><br />
<strong>int volume</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.to_preset</strong>: 导航到预置点。</p>
<p>参数</p>
<p><strong>preset_name</strong>: 预置点名称，类型为字符串。</p>
<p><strong>assisted_relocation</strong>: 是否开启辅助重定位功能，类型为布尔值。</p>
<p>False：关闭（默认值）</p>
<p>True：开启</p>
<p><strong>interact</strong>: 是否开启辅助重定位交互功能，类型为布尔值。</p>
<p>False：关闭（默认值）</p>
<p>True：开启</p>
<p><strong>volume</strong>: 辅助重定位交互音量，类型为整型，默认为50。</p>
<p>返回值：<a>NavigationActionResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation.to_preset('厨房', True , True , 50).response.result == 0:</strong><br />
<strong>print('导航到厨房预置点成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) turn\_on\_navigation（开启（进入）导航模式）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.turn_on_navigation(</strong><br />
<strong>bool outdoor,</strong><br />
<strong>bool assisted_relocation,</strong><br />
<strong>bool interact,</strong><br />
<strong>int volume</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.turn_on_navigation</strong>: 开启导航模式。</p>
<p>参数</p>
<p><strong>outdoor</strong>: 是否为室外环境，类型为布尔值。</p>
<p>False：室内（默认值）</p>
<p>True：室外</p>
<p><strong>assisted_relocation</strong>: 是否开启辅助重定位功能，类型为布尔值。</p>
<p>False：关闭（默认值）</p>
<p>True：开启</p>
<p><strong>interact</strong>: 是否开启辅助重定位交互功能，类型为布尔值。</p>
<p>False：关闭（默认值）</p>
<p>True：开启</p>
<p><strong>volume</strong>: 辅助重定位交互音量，类型为整型，默认为50。</p>
<p>返回值：<a>NavigationActionResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation.turn_on_navigation(False, True , True , 50).response.result == 0:</strong><br />
<strong>print('开启导航模式')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) turn\_off\_relocation（关闭（退出）导航模式）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.turn_off_navigation(</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.turn_off_navigation</strong>: 关闭导航模式。</p>
<p>参数：无</p>
<p>返回值：<a>NavigationActionResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation.turn_off_navigation().response.result == 0:</strong><br />
<strong>print('关闭导航模式')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) navigation\_to\_preset（导航到预置点）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.navigation_to_preset(</strong><br />
<strong>string preset_name</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.navigation_to_preset</strong>: 导航到预置点。</p>
<p>参数</p>
<p><strong>preset_name</strong>: 预置点名称，类型为字符串。</p>
<p>返回值：<a>NavigationActionResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation.navigation_to_preset('厨房').response.result == 0:</strong><br />
<strong>print('导航到厨房预置点成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) navigation\_to\_coordinates（导航到坐标点）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.navigation_to_coordinates(</strong><br />
<strong>double x,</strong><br />
<strong>double y,</strong><br />
<strong>double z,</strong><br />
<strong>double roll,</strong><br />
<strong>double pitch,</strong><br />
<strong>double yaw</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.navigation_to_coordinates</strong>: 导航到坐标点。</p>
<p>参数</p>
<p><strong>x</strong>: 目标点x坐标，类型为浮点型。</p>
<p><strong>y</strong>: 目标点y坐标，类型为浮点型。</p>
<p><strong>z</strong>: 目标点z坐标，类型为浮点型。</p>
<p><strong>roll</strong>: 目标点姿态<strong>roll</strong>，类型为浮点型。</p>
<p><strong>pitch</strong>: 目标点姿态<strong>pitch</strong>，类型为浮点型。</p>
<p><strong>yaw</strong>: 目标点姿态<strong>yaw</strong>，类型为浮点型。</p>
<p>返回值：<a>NavigationActionResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation.navigation_to_coordinates(0,0,0,0,0,0).response.result == 0:</strong><br />
<strong>print('导航到目标点成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) cancel\_navigation（取消导航）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.navigation.cancel_navigation()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.navigation.cancel_navigation</strong>:取消导航。</p>
<p>参数：无</p>
<p>返回值：<a>NavigationActionResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.navigation.cancel_navigation().response.result == 0:</strong><br />
<strong>print('取消导航成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.06 🟡task（任务模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.task.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.task.state</strong>: cyberdog下任务模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.task.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.task.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.task.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.task.set_log</strong>: 设置cyberdog下task模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.task.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下task模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) start（开始任务）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.task.start()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.task.start</strong>:开始任务。</p>
<p>参数：无</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.task.start().state.code == StateCode.fail:</strong><br />
<strong>print('开始任务成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) stop（结束任务）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.task.stop()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.task.stop</strong>:取消导航。</p>
<p>参数：无</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.task.stop().state.code == StateCode.fail:</strong><br />
<strong>print('结束任务成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) block（普通块）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.task.block(</strong><br />
<strong>string id</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.task.block</strong>:设置代码块。</p>
<p>参数：</p>
<p><strong>id</strong>: 当前块id，字符串类型。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.task.block('abc').state.code == StateCode.fail:</strong><br />
<strong>print('触发abc块成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) breakpoint\_block（断点块）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.task.breakpoint_block(</strong><br />
<strong>string id</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.task.breakpoint_block</strong>:设置断点代码块。</p>
<p>说明：当程序执行到该块时，将阻塞如下逻辑功能：</p>
<p>普通块：block</p>
<p>伺服指令相关块：</p>
<p>转向：turn</p>
<p>直行：go_straight</p>
<p>横移：lateral_movevment</p>
<p>前后跳：jump_back_and_forth</p>
<p>小跳行走：small_jump_walking</p>
<p>自变频行走：automatic_frequency_conversion_walking</p>
<p>慢速行走：trot_walking</p>
<p>快跑行走：run_fast_walking</p>
<p>在这里，直到用户点击APP继续按钮后程序才会继续执行，或者调用继续任务接口才会继续。</p>
<p>参数：</p>
<p><strong>id</strong>: 当前块id，字符串类型。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.task.breakpoint_block('abc').state.code == StateCode.fail:</strong><br />
<strong>print('触发abc块成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) recover（继续任务）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.task.recover()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.task.recover</strong>:当任务暂停时，继续直行任务。</p>
<p>说明：该函数将跳出断点的阻塞，使得程序继续运行。</p>
<p>参数：无</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.task.recover().state.code == StateCode.fail:</strong><br />
<strong>print('继续任务成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.07 🟡personnel（人员模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.personnel.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.personnel.state</strong>: cyberdog下人员模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.personnel.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.personnel.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.personnel.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.personnel.set_log</strong>: 设置cyberdog下personnel模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.personnel.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下personnel模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) face\_recognized（识别到目标人员人脸）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
<td></td>
<td></td>
<td></td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.personnel.face_recognized(</strong><br />
<strong>list personnel_ids，</strong><br />
<strong>bool and_operation，</strong><br />
<strong>double duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.personnel.face_recognized</strong>:识别到目标人员人脸。</p>
<p>参数：</p>
<p><strong>names</strong>: 当前识别的目标人员昵称，列表类型。</p>
<p>当列表为空时，识别底库中人员之一即为成功。</p>
<p><strong>and_operation</strong>: 与运算，当<strong>names</strong>字段非空时有效；</p>
<p>默认为False，即为或运算；</p>
<p>True:当前识别到所有目标人员才返回成功。</p>
<p>False:当前识别到目标人员之一就返回成功。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：[30,300]。</p>
<p>返回值：<a>FaceRecognizedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.personnel.face_recognized(['张三']).state.code == StateCode.success:</strong><br />
<strong>print('识别到张三')</strong><br />
<br />
<strong>if cyberdog.personnel.face_recognized(['张三','李四']).state.code == StateCode.success:</strong><br />
<strong>print('识别到张三或李四')</strong><br />
<br />
<strong>if cyberdog.personnel.face_recognized(['张三','李四'], True).state.code == StateCode.success:</strong><br />
<strong>print('识别到张三和李四')</strong><br />
<br />
<strong>ret</strong> = <strong>cyberdog.personnel.face_recognized(['张三','李四'], True)</strong><br />
<strong>if ret.state.code == StateCode.success:</strong><br />
<strong>print('识别到张三和李四')</strong><br />
<strong>if</strong> <strong>ret.dictionary.has_key('张三') and ret.dictionary['张三'].age &lt; 12:</strong><br />
<strong>cyberdog.audio.play('请', ret.response['张三'].username, '同学回家写作业')</strong><br />
if <strong>ret.dictionary.has_key('张三') and ret.response['张三'].emotion &gt; 1:</strong><br />
<strong>cyberdog.audio.play(ret.response['张三'].username, '你看起来很开心')</strong></td>
</tr>
</tbody>
</table></td>
<td></td>
<td></td>
<td></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) voiceprint\_recognized（识别到目标人员声纹）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.personnel.voiceprint_recognized(</strong><br />
<strong>list names，</strong><br />
<strong>bool and_operation，</strong><br />
<strong>double duration,</strong><br />
<strong>int sensitivity</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.personnel.voiceprint_recognized</strong>:识别到目标人员声纹。</p>
<p>参数：</p>
<p><strong>names</strong>: 当前识别的目标人员昵称，列表类型。</p>
<p>当列表为空时，识别底库中人员之一即为成功。</p>
<p><strong>and_operation</strong>: 与运算，当<strong>names</strong>字段非空时有效；</p>
<p>默认为False，即为或运算；</p>
<p>True:当前识别到所有目标人员才返回成功。</p>
<p>False:当前识别到目标人员之一就返回成功。</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：[1, 30]。</p>
<p><strong>sensitivity</strong>: 灵敏度，单位秒（s），类型为整型；</p>
<p>最近一段时间内识别到的均有效。</p>
<p>返回值：<a>VoiceprintRecognized</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.personnel.voiceprint_recognized(['张三']).data:</strong><br />
<strong>print('识别到张三')</strong><br />
<br />
<strong>if cyberdog.personnel.voiceprint_recognized(['张三','李四']).data:</strong><br />
<strong>print('识别到张三或李四')</strong><br />
<br />
<strong>if cyberdog.personnel.voiceprint_recognized(['张三','李四'], True).data:</strong><br />
<strong>print('识别到张三和李四')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.08 🟡audio（语音模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.audio.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.audio.state</strong>: cyberdog下语音模块模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.audio.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.audio.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.audio.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.audio.set_log</strong>: 设置cyberdog下audio模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.audio.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下audio模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) play（播放语音）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.audio.play(</strong><br />
<strong>string message,</strong><br />
<strong>int volume</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.audio.play</strong>:播放语音，该模式为阻塞模式，即播放完语音后才会继续执行下一条。</p>
<p>参数：</p>
<p><strong>message</strong>: 当前播报消息，单行字符串类型（不能包含换行符，或者用\进行转义）。</p>
<p><strong>volume</strong>: 当前及以后播报音量，整型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值：-1</p>
<p>返回值：<a>AudioPlaySeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.audio.play('爱我中国').state.code == StateCode.success:</strong><br />
<strong>print('播报语音成功')</strong><br />
<strong>if cyberdog.audio.play('爱我中国\</strong><br />
<strong>').state.code == StateCode.success:</strong><br />
<strong>print('播报语音成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) instantly\_play（立即播放语音）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.audio.instantly_play(</strong><br />
<strong>string message,</strong><br />
<strong>int volume</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.audio. instantly_play</strong>: 立即播放语音，该模式为非阻塞且抢占模式，即播放语音的同时继续向下执行，如果当前正在播放语音则打断当前语音并播放新的语音。</p>
<p>参数：</p>
<p><strong>message</strong>: 当前播报消息，单行字符串类型（不能包含换行符，或者用\进行转义）。</p>
<p><strong>volume</strong>: 当前及以后播报音量，整型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值：-1</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.audio.instantly_play('爱我中国').state.code == StateCode.success:</strong><br />
<strong>print('播报语音成功')</strong><br />
<strong>if cyberdog.audio.instantly_play('爱我中国\</strong><br />
<strong>').state.code == StateCode.success:</strong><br />
<strong>print('播报语音成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) offline\_play（播放离线语音）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.audio.offline_play(</strong><br />
<strong>int audio_id,</strong><br />
<strong>int volume</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.audio.offline_play</strong>:播放离线语音，该模式为阻塞模式，即播放完语音后才会继续执行下一条。</p>
<p>参数：</p>
<p><strong>audio_id</strong>: 当前播报消息，整数类型，只支持以下值：</p>
<p>4000：汪汪（细声）</p>
<p>4001：汪汪（粗声）</p>
<p>6000：音乐1</p>
<p>6001：音乐1</p>
<p><strong>volume</strong>: 当前及以后播报音量，整型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值：-1</p>
<p>返回值：<a>AudioPlaySeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.audio.offline_play(4000).state.code == StateCode.success:</strong><br />
<strong>print('播报语音成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) offline\_instantly\_play（立即播放离线语音）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.audio.offline_instantly_play(</strong><br />
<strong>int audio_id,</strong><br />
<strong>int volume</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.audio.offline_instantly_play</strong>: 立即播放离线语音，该模式为非阻塞且抢占模式，即播放语音的同时继续向下执行，如果当前正在播放语音则打断当前语音并播放新的语音。</p>
<p>参数：</p>
<p><strong>audio_id</strong>: 当前播报消息，整数类型，只支持以下值：</p>
<p>4000：汪汪（细声）</p>
<p>4001：汪汪（粗声）</p>
<p>6000：音乐1</p>
<p>6001：音乐1</p>
<p><strong>volume</strong>: 当前及以后播报音量，整型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值：-1</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.audio.offline_instantly_play(4000).state.code == StateCode.success:</strong><br />
<strong>print('播报语音成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_volume（获取播放音量）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.audio.get_volume()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.audio.get_volume</strong>: 获取播放音量。</p>
<p>参数：无</p>
<p>返回值：<a>AudioGetVolumeSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>audio_volume = cyberdog.audio.get_volume()</strong><br />
<strong>if audio_volume.state.code == StateCode.success:</strong><br />
<strong>print('获取音量成功，当前音量为：', audio_volume.response.volume)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) set\_volume（设置播放音量）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.audio.set_volume(</strong><br />
<strong>int volume</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.audio.set_volume</strong>: 设置播放音量。</p>
<p>参数：</p>
<p><strong>volume</strong>: 当前及以后播报音量，整型。</p>
<p>合法值约束：<a>铁蛋能力集参数约束表</a>。</p>
<p>默认值：-1</p>
<p>返回值：<a>AudioSetVolumeSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.audio.set_volume(10).response.success:</strong><br />
<strong>print('设置播报音量成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.09 🟡led（led模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.led.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.led.state</strong>: cyberdog下led模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.led.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.led.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.led.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.led.set_log</strong>: 设置cyberdog下led模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.led.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下led模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) play（播放系统灯效）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.led.play(</strong><br />
<strong>int target,</strong><br />
<strong>int effect</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.led.play</strong>:播放播放系统灯效，一旦触发就会一直保持该灯效，直到下一个灯效请求来临。</p>
<blockquote>
<p><strong>注意：</strong></p>
</blockquote>
<p>当LED被高级别模块（cyberdog_manager、低功耗、低电量等）调用时，可视化编程请求灯效会失败。</p>
<p>当任务结束时会释放LED控制权限，交由系统其他模块控制。</p>
<p>参数：</p>
<p><strong>target</strong>: 当前控制的目标灯，整型。</p>
<p>LedConstraint.target_*。</p>
<p>约束详情参见<a>LED参数约束</a>。</p>
<p><strong>effect</strong>: 当前灯效，整型，合法值如下（16进制给出）：</p>
<p>LedConstraint.system_effect_line_* 或</p>
<p>LedConstraint.system_effect_mini_*。</p>
<p>约束详情参见<a>LED参数约束</a>。</p>
<p>返回值：<a>LedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.led.play(LedConstraint.target_head, LedConstraint.system_effect_line_red_on).state.code == StateCode.success:</strong><br />
<strong>print('控制头灯常亮成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) play\_rgb（播放RGB灯效）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.led.play_rgb(</strong><br />
<strong>int target,</strong><br />
<strong>int effect,</strong><br />
<strong>int r,</strong><br />
<strong>int g,</strong><br />
<strong>int b</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.led.play</strong>:播放RGB灯效，一旦触发就会一直保持该灯效，直到下一个灯效请求来临。</p>
<blockquote>
<p><strong>注意：</strong></p>
</blockquote>
<p>当LED被高级别模块（cyberdog_manager、低功耗、低电量等）调用时，可视化编程请求灯效会失败。</p>
<p>当任务结束时会释放LED控制权限，交由系统其他模块控制。</p>
<p>参数：详情参见<a>LED参数约束</a>。</p>
<p><strong>target</strong>: 当前控制的目标灯，整型。</p>
<p>LedConstraint.target_*。</p>
<p>约束详情参见<a>LED参数约束</a>。</p>
<p><strong>effect</strong>: 当前灯效，整型，合法值如下（16进制给出）：</p>
<p>LedConstraint.effect_line_* 或</p>
<p>LedConstraint.effect_mini_*。</p>
<p>约束详情参见<a>LED参数约束</a>。</p>
<p><strong>r</strong>:当前红色通道的灰度值，整型，取值范围[0,255]</p>
<p><strong>g</strong>:当前绿色通道的灰度值，整型，取值范围[0,255]</p>
<p><strong>b</strong>:当前蓝色通道的灰度值，整型，取值范围[0,255]</p>
<p>返回值：<a>LedSeviceResponse</a></p>
<p>备注：色板参见<a href="https://www.rapidtables.com/web/color/RGB_Color.html">RGB</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.led.play_rgb(LedConstraint.target_head, LedConstraint.effect_line_breath_fast, 255, 255, 255).state.code == StateCode.success:</strong><br />
<strong>print('控制头灯以白色展示快速呼吸成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) freed（释放指定设备控制权）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.led.freed(</strong><br />
<strong>int target</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.led.freed</strong>:释放指定设备控制权。</p>
<p>参数：</p>
<p><strong>target</strong>: 当前控制的目标灯，整型。</p>
<p>LedConstraint.target_*。</p>
<p>约束详情参见<a>LED参数约束</a>。</p>
<p>返回值：<a>LedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.led.freed(LedConstraint.target_head).state.code == StateCode.success:</strong><br />
<strong>print('释放头部灯带控制权成功')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.10 🟡bms（电池管理系统模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.bms.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.bms.state</strong>: cyberdog下电池管理系统模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.bms.state.code == StateCode.fail &amp;&amp; :</strong><br />
<strong>print(cyberdog.bms.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.bms.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.bms.data</strong>: cyberdog下电池管理系统模块状态获取接口名称。</p>
<p>类型：<a>BmsStatus</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.bms.data.batt_soc &lt; 20:</strong><br />
<strong>print('当前机器人电量低于20%')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.bms.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.bms.set_log</strong>: 设置cyberdog下bms模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.bms.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下bms模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_data（获取最新数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.bms.get_data(</strong><br />
<strong>int</strong> <strong>timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.bms.get_data</strong>:获取最新电池数据。</p>
<p>参数：</p>
<p><strong>timeout：</strong>当前获取动作超时限制（单位：秒），整型。</p>
<p>默认值：5。</p>
<p>返回值：<a>BmsStatus</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.bms.get_data().batt_soc &lt; 20:</strong><br />
<strong>print('当前机器人电量低于20%')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.11 🟡touch（触摸板模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.touch.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.touch.state</strong>: cyberdog下触摸板模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.touch.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.touch.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.touch.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.touch.data</strong>: cyberdog下触摸板模块状态获取接口名称。</p>
<p>类型：<a>TouchStatus</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.touch.data.touch_state:</strong><br />
<strong>print('当前机器人触摸板已被触发')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.touch.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.touch.set_log</strong>: 设置cyberdog下touch模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.touch.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下touch模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_data（获取最新数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.touch.get_data(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.touch.get_data</strong>:获取最新touch数据。</p>
<p>参数：</p>
<p><strong>timeout：</strong>当前获取动作超时限制（单位：秒），整型。</p>
<p>默认值：5。</p>
<p>返回值：<a>TouchStatus</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.touch.get_data().touch_state:</strong><br />
<strong>print('当前机器人触摸板已被触发')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.12 🟡gps（全球定位系统模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gps.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gps.state</strong>: cyberdog模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gps.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.gps.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gps.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gps.data</strong>: cyberdog下全球定位系统模块状态获取接口名称。</p>
<p>类型：<a>GpsPayload</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gps.data.num_sv &gt; 3:</strong><br />
<strong>print('当前机器人经纬度为:', cyberdog.network.data.lon, ',', cyberdog.network.data.lat)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gps.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gps.set_log</strong>: 设置cyberdog下touch模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gps.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下gps模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_data（获取最新数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gps.get_data(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gps.get_data</strong>:获取最新<strong>gps</strong>数据。</p>
<p>参数：</p>
<p><strong>timeout：</strong>当前获取动作超时限制（单位：秒），整型。</p>
<p>默认值：5。</p>
<p>返回值：<a>GpsPayload</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gps.get_data().num_sv &gt; 3:</strong><br />
<strong>print('当前机器人经纬度为:', cyberdog.network.data.lon, ',', cyberdog.network.data.lat)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.13 🟡tof（激光测距模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.tof.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.tof.state</strong>: cyberdog下激光测距模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.tof.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.tof.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.tof.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.tof.data</strong>: cyberdog下激光测距模块状态获取接口名称。</p>
<p>类型：<a>TofPayload</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.tof.data)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.tof.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.tof.set_log</strong>: 设置cyberdog下tof模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.tof.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下tof模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_data（获取最新数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.tof.get_data(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.tof.get_data</strong>:获取最新<strong>tof</strong>数据。</p>
<p>参数：</p>
<p><strong>timeout：</strong>当前获取动作超时限制（单位：秒），整型。</p>
<p>默认值：5。</p>
<p>返回值：<a>TofPayload</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.tof.get_data())</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.14 🟡lidar（雷达模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.lidar.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.lidar.state</strong>: cyberdog下雷达模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.lidar.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.lidar.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.lidar.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.lidar.data</strong>: cyberdog下雷达模块状态获取接口名称。</p>
<p>类型：<a>LaserScan</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.lidar.data)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.lidar.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.lidar.set_log</strong>: 设置cyberdog下lidar模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.lidar.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下lidar模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_data（获取最新数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.lidar.get_data(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.lidar.get_data</strong>:获取最新<strong>lidar</strong>数据。</p>
<p>参数：</p>
<p><strong>timeout：</strong>当前获取动作超时限制（单位：秒），整型。</p>
<p>默认值：5。</p>
<p>返回值：<a>LaserScan</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.lidar.get_data())</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.15 🟡ultrasonic（超声波模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.ultrasonic.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.ultrasonic.state</strong>: cyberdog下超声波模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.ultrasonic.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.ultrasonic.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.ultrasonic.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.ultrasonic.data</strong>: cyberdog下超声波模块状态获取接口名称。</p>
<p>类型：<a>Range</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.ultrasonic.data)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.ultrasonic.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.ultrasonic.set_log</strong>: 设置cyberdog下ultrasonic模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.ultrasonic.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下ultrasonic模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_data（获取最新数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.ultrasonic.get_data(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.ultrasonic.get_data</strong>:获取最新<strong>ultrasonic</strong>数据。</p>
<p>参数：</p>
<p><strong>timeout：</strong>当前获取动作超时限制（单位：秒），整型。</p>
<p>默认值：5。</p>
<p>返回值：<a>Range</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.ultrasonic.get_data())</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.16 🟡odometer（里程计模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.odometer.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.odometer.state</strong>: cyberdog下里程计模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.odometer.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.odometer.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.odometer.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.odometer.data</strong>: cyberdog下里程计模块状态获取接口名称。</p>
<p>类型：<a>Odometry</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.odometer.data)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

🟢 set\_log（设置日志）

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.odometer.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.odometer.set_log</strong>: 设置cyberdog下odometer模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.odometer.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下odometer模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_data（获取最新数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.odometer.get_data(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.odometer.get_data</strong>:获取最新<strong>odometer</strong>数据。</p>
<p>参数：</p>
<p><strong>timeout：</strong>当前获取动作超时限制（单位：秒），整型。</p>
<p>默认值：5。</p>
<p>返回值：<a>Range</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.odometer.get_data())</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.17 🟡imu（惯导模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.imu.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.imu.state</strong>: cyberdog下惯导模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.imu.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.imu.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) data（数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.imu.data</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.imu.data</strong>: cyberdog下惯导模块状态获取接口名称。</p>
<p>类型：<a>Imu</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.imu.data)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.imu.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.imu.set_log</strong>: 设置cyberdog下imu模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.imu.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下imu模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_data（获取最新数据）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.imu.get_data(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.imu.get_data</strong>:获取最新<strong>imu</strong>数据。</p>
<p>参数：</p>
<p><strong>timeout：</strong>当前获取动作超时限制（单位：秒），整型。</p>
<p>默认值：5。</p>
<p>返回值：<a>Imu</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print(cyberdog.imu.get_data())</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.18 🟡gesture（手势识别模块）**

手势识别功能性能参考[连续手势识别测试方法](https://xiaomi.f.mioffice.cn/docx/doxk4yqqAvVDsdaNnRmbTXKmqxb) 文档。

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gesture.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gesture.state</strong>: cyberdog下手势识别模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gesture.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.gesture.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gesture.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gesture.set_log</strong>: 设置cyberdog下gesture模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gesture.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下gesture模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) recognized（开始识别并识别到任意手势）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gesture.recognized(</strong><br />
<strong>int duration,</strong><br />
<strong>int sensitivity</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gesture.recognized</strong>:识别到手势。</p>
<p>参数：</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为整型；</p>
<p>合法值约束：[30,300]。</p>
<p><strong>sensitivity</strong>: 灵敏度，单位秒（s），类型为整型；</p>
<p>最近一段时间内识别到的均有效。</p>
<p>返回值：<a>GestureRecognizedMessageResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gesture.recognized().data.pulling_hand_or_two_fingers_in:</strong><br />
<strong>print('识别到底库人员的手掌拉近手势')</strong><br />
<br />
<strong>if cyberdog.gesture.recognized().data.pulling_hand_or_two_fingers_in:</strong><br />
<strong>print('识别到手掌拉近手势') </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) turn\_on\_recognition（打开识别手势功能）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gesture.turn_on_recognition(</strong><br />
<strong>int duration</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gesture.turn_on_recognition</strong>:打开识别手势功能。</p>
<p>参数：</p>
<p><strong>duration</strong>: 期望时间，单位秒（s），类型为整型；</p>
<p>合法值约束：[30,300]。</p>
<p>返回值：<a>GestureRecognizedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gesture.turn_on_recognition(300).response.code == 0:</strong><br />
<strong>print('打开手势识别功能成功') </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) turn\_off\_recognition（关闭识别手势功能）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gesture.turn_off_recognition(</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gesture. turn_off_recognition</strong>:关闭识别手势功能。</p>
<p>参数：无</p>
<p>返回值：<a>GestureRecognizedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gesture.turn_off_recognition().response.code == 0:</strong><br />
<strong>print('打开手势识别功能失败') </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) recognized\_designated\_gesture（识别到指定手势）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gesture.recognized_designated_gesture(</strong><br />
<strong>int timeout,</strong><br />
<strong>int gesture_type</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gesture.recognized_designated_gesture</strong>:识别到指定手势。</p>
<p>参数：</p>
<p><strong>timeout</strong>: 超时时间，单位秒（s），类型为整型；</p>
<p>合法值约束：[1,300]。</p>
<p><strong>gesture_type</strong>: 手势类型，类型为整型，详情参见<a>GestureType</a>；</p>
<p>指定识别目标手势。</p>
<p>返回值：<a>GestureRecognizedMessageResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
// 以识别到手掌拉近为例：<br />
<strong>if cyberdog.gesture.recognized_designated_gesture(60, 1).data.pulling_hand_or_two_fingers_in:</strong><br />
<strong>print('识别到底库人员的手掌拉近手势')</strong><br />
// 等价于下面的调用方式<br />
<strong>if cyberdog.gesture.recognized_designated_gesture(60, GestureType.pulling_hand_or_two_fingers_in).data.pulling_hand_or_two_fingers_in:</strong><br />
<strong>print('识别到底库人员的手掌拉近手势')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) recognized\_any\_gesture（识别到任意手势）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.gesture.recognized_any_gesture(</strong><br />
<strong>int timeout,</strong><br />
<strong>int sensitivity</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.gesture. recognized_any_gesture</strong>:识别到任意手势。</p>
<p>参数：</p>
<p><strong>timeout</strong>: 超时时间，单位秒（s），类型为整型；</p>
<p>合法值约束：[30,300]。</p>
<p><strong>sensitivity</strong>: 灵敏度，单位秒（s），类型为整型；</p>
<p>最近一段时间内识别到的均有效。</p>
<p>返回值：<a>GestureRecognizedMessageResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.gesture.recognized_any_gesture(60, 1).data.pulling_hand_or_two_fingers_in:</strong><br />
<strong>print('识别到底库人员的手掌拉近手势')</strong><br />
<br />
<strong>if cyberdog.gesture.recognized_any_gesture(60, 1).data.pulling_hand_or_two_fingers_in:</strong><br />
<strong>print('识别到手掌拉近手势') </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.19 🟡skeleton（骨骼点识别模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.skeleton.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.skeleton.state</strong>: cyberdog下骨骼点识别模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.skeleton.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.skeleton.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.skeleton.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.skeleton.set_log</strong>: 设置cyberdog下skeleton模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.skeleton.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下skeleton模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) sports\_recognition（运动识别）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.skeleton.sports_recognition(</strong><br />
<strong>int sport_type,</strong><br />
<strong>int counts,</strong><br />
<strong>int timeout,</strong><br />
<strong>bool interact,</strong><br />
<strong>bool instantly,</strong><br />
<strong>int volume</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.skeleton. sports_recognition</strong>:运动识别。</p>
<p>参数：</p>
<p><strong>sport_type</strong>: 识别类型，类型为整型，约束参见<a>SkeletonType</a>；</p>
<p>1 # 深蹲</p>
<p>2 # 高抬腿</p>
<p>3 # 仰卧起坐</p>
<p>4 # 俯卧撑</p>
<p>5 # 平板支撑</p>
<p>6 # 开合跳</p>
<p><strong>counts</strong>: 动作个数，类型为整型；</p>
<p>申请做动作的个数，从1开始。</p>
<p><strong>timeout</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：[5,300]。</p>
<p><strong>interact</strong>: 是否开启交互功能，类型为布尔值。</p>
<p>False：关闭</p>
<p>True：开启（默认值）</p>
<p><strong>instantly</strong>: 是否开启立即交互立即交互功能，类型为布尔值。</p>
<p>False：关闭</p>
<p>True：开启（默认值）</p>
<p><strong>volume</strong>: 辅助重定位交互音量，类型为整型，默认为50。</p>
<p>返回值：<a>DefineSkeletonRecognizedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.skeleton.sports_recognition(SkeletonType.squat, 10, 60, True, True, 50).response.result == 0:</strong><br />
<strong>print('开启骨骼点深蹲10个检测识别功能成功') </strong></td>
</tr>
</tbody>
</table>
<table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.skeleton.sports_recognition(1, 10, 60, True, True, 50).response.result == 0:</strong><br />
<strong>print('开启骨骼点深蹲10个检测识别功能成功') </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) turn\_on\_recognition（打开识别骨骼点功能）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.skeleton.turn_on_recognition(</strong><br />
<strong>int sport_type,</strong><br />
<strong>int counts,</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.skeleton. turn_on_recognition</strong>:打开识别骨骼（点）功能。</p>
<p>参数：</p>
<p><strong>sport_type</strong>: 识别类型，类型为整型，约束参见<a>SkeletonType</a>；</p>
<p>1 # 深蹲</p>
<p>2 # 高抬腿</p>
<p>3 # 仰卧起坐</p>
<p>4 # 俯卧撑</p>
<p>5 # 平板支撑</p>
<p>6 # 开合跳</p>
<p><strong>counts</strong>: 动作个数，类型为整型；</p>
<p>申请做动作的个数，从1开始。</p>
<p><strong>timeout</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：[5,300]。</p>
<p>返回值：<a>DefineSkeletonRecognizedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.skeleton.turn_on_recognition(SkeletonType.squat, 10, 60).response.result == 0:</strong><br />
<strong>print('开启骨骼点深蹲10个检测识别功能成功') </strong></td>
</tr>
</tbody>
</table>
<table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.skeleton.turn_on_recognition(1, 10, 60).response.result == 0:</strong><br />
<strong>print('开启骨骼点深蹲10个检测识别功能成功') </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) turn\_off\_recognition（关闭识别骨骼点功能）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.skeleton.turn_off_recognition()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.skeleton. turn_off_recognition</strong>:关闭识别骨骼（点）功能。</p>
<p>参数：无</p>
<p>返回值：<a>DefineSkeletonRecognizedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.skeleton.turn_off_recognition().response.result == 0:</strong><br />
<strong>print('关闭骨骼点识别功能成功') </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) blocking\_recognized（阻塞式识别到）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.skeleton.blocking_recognized(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.skeleton.blocking_recognized</strong>:打开识别骨骼（点）功能。</p>
<p>参数：</p>
<p><strong>timeout</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：[5,300]。</p>
<p>返回值：<a>SkeletonRecognizedMessageResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
// 交互，<strong>print换位语音接口就可以实现交互</strong><br />
<strong>print('当前运动计数为', cyberdog.skeleton.blocking_recognized().response.counts，‘个’)</strong><br />
<br />
<strong>print('当前运动时长为', cyberdog.skeleton.blocking_recognized().response.duration，‘秒’) </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) instant\_recognized（瞬时式识别到）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.skeleton.instant_recognized()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.skeleton.instant_recognized</strong>:打开识别骨骼（点）功能。</p>
<p>参数：无</p>
<p>返回值：<a>SkeletonRecognizedMessageResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
// 判断当前算法是否开启<br />
if (<strong>cyberdog.skeleton.instant_recognized().response.algo_switch == 0</strong>) // 开启<br />
if (<strong>cyberdog.skeleton.instant_recognized().response.algo_switch == 1</strong>) // 关闭</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.4.20 🟡train（训练模块）**

**[🟣](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#Qp608Q) state（状态）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.train.state</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.train.state</strong>: cyberdog下训练模块状态获取接口名称。</p>
<p>类型：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.train.state.code == StateCode.fail:</strong><br />
<strong>print(cyberdog.train.state.describe)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**🟢 set\_log（设置日志）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.train.set_log(</strong><br />
<strong>bool log)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.train.set_log</strong>: 设置cyberdog下train模块日志。</p>
<p>参数：</p>
<p><strong>log</strong>: 设置日志状态，布尔类型；</p>
<p>True：开启cyberdog模块日志；</p>
<p>False：关闭cyberdog模块日志。</p>
<p>返回值：<a>State</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>if cyberdog.train.set_log(False).code == StateCode.success:</strong><br />
<strong>print('已关闭cyberdog下train模块日志')</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) get\_training\_words\_set（获取训练词集合）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.train.get_training_words_set(</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.train.get_training_words_set</strong>:获取训练词集合。</p>
<p>参数：无</p>
<p>返回值：<a>TrainingWordsRecognizedSeviceResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog. train.get_training_words_set()</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) training\_words\_recognized（识别到训练词）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>cyberdog.train.training_words_recognized(</strong><br />
<strong>int timeout</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>cyberdog.train.get_training_words_set</strong>:获取训练词集合。</p>
<p>参数：</p>
<p><strong>timeout</strong>: 期望时间，单位秒（s），类型为浮点型；</p>
<p>合法值约束：[5,300]。</p>
<p>返回值：<a>TrainingWordsRecognizedMessageResponse</a></p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>print('识别到训练词：'，cyberdog. train.training_words_recognized().response.trigger)</strong><br />
<br />
if <strong>cyberdog. train.training_words_recognized().response.trigger == '张三'：</strong><br />
<strong>print('识别到训练词为：张三') </strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.5 内置能力接口约束**

**4.5.1 🟡choreographer（编舞模块）**

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) moonwalk（太空步）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>choreographer.moonwalk(</strong><br />
<strong>float x_velocity,</strong><br />
<strong>float y_velocity,</strong><br />
<strong>float stride,</strong><br />
<strong>int number</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>choreographer.moonwalk</strong>:对太空步进行编舞。</p>
<p>参数：</p>
<p><strong>x_velocity</strong>: X轴速度，单位米每秒（m/s），类型为浮点型；</p>
<p>合法值约束：[-0.08, 0.08]。</p>
<p><strong>y_velocity</strong>: Y轴速度，单位米每秒（m/s），类型为浮点型；</p>
<p>合法值约束：[-0.05, 0.05]。</p>
<p><strong>stride</strong>: 不幅，单位米（m），类型为浮点型；</p>
<p>合法值约束：(0,0.065]。</p>
<p><strong>number</strong>: 太空步重复次数，类型为整型；</p>
<p>合法值约束：[1,10]。</p>
<p>返回值：无</p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>choreographer.moonwalk(0.05, 0.03, 0.03, 2)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) push\_up（俯卧撑）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>choreographer.push_up(</strong><br />
<strong>int frequency,</strong><br />
<strong>int number</strong><br />
<strong>)</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>choreographer.moonwalk</strong>:对太空步进行编舞。</p>
<p>参数：</p>
<p><strong>frequency</strong>: 不幅，单位次每分钟，类型为整型；</p>
<p>合法值约束：[10, 60]。</p>
<p><strong>number</strong>: 俯卧撑重复次数，类型为整型；</p>
<p>合法值约束：[2,10]。</p>
<p>返回值：无</p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>choreographer.push_up(10, 3)</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**4.5.2 🟡dancer（舞蹈家模块）**

**[🟢](https://xiaomi.f.mioffice.cn/docs/dock4nUNWHVve526QMxnT4b0taf#DTXadn) dance（跳舞）**

<table>
<tbody>
<tr class="odd">
<td>协议约束</td>
<td>协议说明</td>
<td>举例</td>
</tr>
<tr class="even">
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>dancer.dance()</strong></td>
</tr>
</tbody>
</table></td>
<td><p>接口名称</p>
<p><strong>dancer. dance</strong>:按照编舞进行跳舞。</p>
<p>参数：无</p>
<p>返回值：无</p></td>
<td><table>
<tbody>
<tr class="odd">
<td><br />
<strong>dancer.dance()</strong></td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

**五、系统能力集**

目前仅仅支持python3 time模块的其他功能，理论上支持python3所有其他功能，但是需要在图形化编程引擎做微调。
