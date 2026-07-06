#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
解析 DLGC0512.SPY 项目文件，生成零基础说明文档。
"""
import json
import re
import xml.etree.ElementTree as ET

SPY_FILE = 'DLGC0512.SPY'
OUT_MD = 'DLGC0512说明文档.md'
NS = 'https://developers.google.com/blockly/xml'


def qname(tag):
    return f'{{{NS}}}{tag}'


def load_spy():
    with open(SPY_FILE, 'r', encoding='utf-8') as f:
        return json.load(f)


def get_field(block, name):
    el = block.find(f'{qname("field")}[@name="{name}"]')
    return el.text if el is not None else None


def get_shadow_num(block, value_name):
    v = block.find(f'{qname("value")}[@name="{value_name}"]')
    if v is None:
        return None
    # 优先用 block，再用 shadow
    num_el = v.find(f'{qname("block")}/{qname("field")}[@name="NUM"]')
    if num_el is None:
        num_el = v.find(f'{qname("shadow")}/{qname("field")}[@name="NUM"]')
    return num_el.text if num_el is not None else None


def summarize_block(block):
    """把一块积木转成简短中文说明"""
    t = block.get('type')
    if t == 'controls_wait':
        return f"等待 {get_shadow_num(block, 'time')} 秒"
    elif t == 'setServoTime':
        servo = get_field(block, 'PORT')
        angle = get_shadow_num(block, 'angle')
        ms = get_shadow_num(block, 'ms')
        return f"舵机 {servo} 转到 {angle}°，用 {ms} 毫秒完成"
    elif t == 'setServo':
        servo = get_field(block, 'PORT')
        angle = get_shadow_num(block, 'angle')
        return f"设置舵机 {servo} 角度为 {angle}°"
    elif t in ('code_procedures_callnoreturn', 'procedures_callnoreturn'):
        mut = block.find(qname('mutation'))
        name = mut.get('name') if mut is not None else '(?)'
        args = []
        if mut is not None:
            for i, arg in enumerate(mut.findall(qname('arg'))):
                aname = arg.get('name')
                val = get_shadow_num(block, f'ARG{i}')
                args.append(f'{aname}={val}')
        return f"调用「{name}」({', '.join(args)})"
    elif t == 'procedures_defnoreturn':
        return f"定义函数：{get_field(block, 'NAME')}"
    elif t == 'sc_main':
        return "【主程序开始】"
    else:
        return f"[{t}]"


def flatten_next_chain(start_block):
    """沿 <next> 链条展开积木序列"""
    seq = []
    b = start_block
    while b is not None:
        seq.append(b)
        nxt = b.find(qname('next'))
        b = nxt.find(qname('block')) if nxt is not None else None
    return seq


def extract_functions(root):
    """提取所有自定义函数及其步骤"""
    funcs = {}
    for top in root.findall(qname('block')):
        if top.get('type') != 'procedures_defnoreturn':
            continue
        name = get_field(top, 'NAME')
        stmt = top.find(f'{qname("statement")}[@name="STACK"]')
        steps = []
        if stmt is not None:
            first = stmt.find(qname('block'))
            if first is not None:
                steps = [summarize_block(b) for b in flatten_next_chain(first)]
        funcs[name] = steps
    return funcs


def extract_main_flow(root):
    """提取主程序流程"""
    for top in root.findall(qname('block')):
        if top.get('type') == 'sc_main':
            nxt = top.find(qname('next'))
            if nxt is None:
                return []
            first = nxt.find(qname('block'))
            if first is None:
                return []
            return [summarize_block(b) for b in flatten_next_chain(first)]
    return []


def parse_user_py(text):
    """按函数拆分 user.py，返回 {函数名: [行列表]}"""
    lines = text.splitlines()
    groups = {}
    current = '__顶部全局代码__'
    groups[current] = []
    pattern = re.compile(r'^def\s+(\w+)\s*\(')
    for line in lines:
        m = pattern.match(line)
        if m:
            current = m.group(1)
            groups[current] = []
        groups[current].append(line)
    return groups


def explain_function_code(name, lines):
    """对 user.py 中的函数代码做极简解释"""
    body = '\n'.join(lines)
    # 提取关键信息
    info = []

    # 参数
    first = lines[0] if lines else ''
    param_match = re.search(r'def\s+\w+\s*\((.*?)\):', first)
    if param_match:
        params = [p.strip() for p in param_match.group(1).split(',') if p.strip()]
        if params:
            info.append(f"参数：{', '.join(params)}")

    # 关键动作识别
    keywords = {
        'setmotor': '控制左右电机',
        'xyrobot.setMotor': '控制电机',
        'getADC': '读取光电/传感器数值',
        'time.sleep': '等待一段时间',
        'Servo': '控制舵机',
        'display': '在 OLED 小屏幕上显示内容',
        '_calc_pid': '计算 PID 纠偏值',
        '_read_sensor_error': '读取巡线误差',
        '_motor_control': '根据 PID 输出控制电机',
        '巡线': '沿着黑线/白线行驶',
        '转弯': '控制左右轮差速转弯',
        '走距离': '按指定距离行驶',
        '走编码': '按编码器计数行驶',
        '夹子': '控制机械臂/夹子',
        '机械臂': '控制机械臂升降',
    }
    for kw, desc in keywords.items():
        if kw in body:
            info.append(desc)

    # 去除重复
    seen = set()
    unique = []
    for i in info:
        if i not in seen:
            seen.add(i)
            unique.append(i)
    return unique


def build_doc(data):
    root = ET.fromstring(data['main.xml'])
    config = json.loads(data['config.json'])
    ftp = json.loads(data['ftpip.json'])
    user_py_groups = parse_user_py(data['user.py'])
    main_flow = extract_main_flow(root)
    funcs = extract_functions(root)

    lines = []
    lines.append('# DLGC0512 项目零基础说明文档')
    lines.append('')
    lines.append('> 本文档由程序自动从 `.SPY` 项目文件生成，帮助你零基础理解这个机器人项目。')
    lines.append('')

    # 1. 项目概览
    lines.append('## 一、这个项目是做什么的？')
    lines.append('')
    lines.append('这是一个 **XYRobot-S3 机器人** 的图形化积木编程项目。')
    lines.append('')
    lines.append('从文件里的函数名和动作来看，这个机器人大概要完成的任务有：')
    lines.append('- 沿着地上的黑线/白线自动行驶（**巡线**）')
    lines.append('- 在路口转弯、选择路线（**三岔路口**）')
    lines.append('- 用机械臂/夹子搬运东西（**球、杯、易拉罐、泡沫方块**）')
    lines.append('- 把物品放到指定位置（**放杯左 / 放杯右**）')
    lines.append('')
    lines.append('可以理解为：这是一个 **自动搬运机器人比赛项目**。')
    lines.append('')

    # 2. 项目基本信息
    lines.append('## 二、项目基本信息')
    lines.append('')
    lines.append('| 项目 | 内容 | 说明 |')
    lines.append('|---|---|---|')
    lines.append(f"| 项目名称 | {config.get('project_Name','')} | 保存的文件名 |")
    lines.append(f"| 主控板 | {config.get('board','')} | 机器人主控芯片型号 |")
    lines.append(f"| 编程模式 | {config.get('mode','')} | block = 图形化积木 |")
    lines.append(f"| 难度等级 | {config.get('level','')} | 初级 |")
    lines.append(f"| 代码行数 | {config.get('code_lineNumber','')} | 生成代码的行数 |")
    lines.append(f"| FTP 地址 | {ftp.get('ftpip','')} | 给机器人传程序的地址 |")
    lines.append(f"| FTP 密码 | {ftp.get('pwd','')} | 传程序用的密码 |")
    lines.append('')

    # 3. 文件结构
    lines.append('## 三、项目里有哪些文件？')
    lines.append('')
    lines.append('一个 `.SPY` 文件其实是个压缩包，里面包含 4 个部分：')
    lines.append('')
    lines.append('| 文件名 | 作用 |')
    lines.append('|---|---|')
    lines.append('| `config.json` | 项目配置（名称、主板、难度等） |')
    lines.append('| `main.xml` | 你在积木界面里拖出来的积木程序 |')
    lines.append('| `user.py` | 积木自动转换成的 Python 代码，真正下载到机器人里运行 |')
    lines.append('| `ftpip.json` | 传程序到机器人时用的网络地址和密码 |')
    lines.append('')

    # 4. 主程序流程
    lines.append('## 四、主程序流程（机器人启动后会按这个顺序执行）')
    lines.append('')
    lines.append('主程序就是机器人一开机就开始跑的步骤。下面是它按顺序做的事：')
    lines.append('')
    for i, step in enumerate(main_flow, 1):
        lines.append(f"{i}. {step}")
    lines.append('')
    lines.append('（主程序很长，后面大部分是调用各种任务函数。你可以把主程序理解为「任务清单」，真正的动作封装在下面的函数里。）')
    lines.append('')

    # 5. 自定义函数分类
    lines.append('## 五、自定义函数一览（按任务分类）')
    lines.append('')
    lines.append('函数就像是「小本领」，主程序需要哪个本领就调用哪个。')
    lines.append('')

    # 分类
    categories = {
        '球/杯任务': [],
        '易拉罐任务': [],
        '泡沫方块任务': [],
        '路线/三岔口': [],
        '基础动作': [],
        '初始化与设置': [],
        '其他': []
    }
    for name in funcs.keys():
        if name.startswith('球') and '杯' in name:
            categories['球/杯任务'].append(name)
        elif '易拉罐' in name:
            categories['易拉罐任务'].append(name)
        elif '泡沫' in name:
            categories['泡沫方块任务'].append(name)
        elif '三岔' in name or name.startswith('a') and ('走' in name or re.search(r'a\d+$', name)):
            categories['路线/三岔口'].append(name)
        elif name in ('设置', '设置机器参数', '舵机初始化', '设置机械臂_升', '设置机械臂_降',
                      '抬起', '降下', '夹泡沫', '松开泡沫', '抬起泡沫', '降下泡沫',
                      '后退夹易拉罐', '降下夹易拉罐', '闸口', '闸口去终点',
                      '准备夹泡沫', '放杯左', '放杯右'):
            categories['基础动作'].append(name)
        elif name in ('设置', '设置机器参数', '舵机初始化'):
            categories['初始化与设置'].append(name)
        else:
            categories['其他'].append(name)

    for cat, names in categories.items():
        if not names:
            continue
        lines.append(f"### {cat}")
        lines.append('')
        for name in names:
            steps = funcs[name]
            lines.append(f"#### `{name}`")
            lines.append('')
            if not steps:
                lines.append('（这个函数在积木里暂时没有具体步骤，可能是占位用的。）')
            else:
                lines.append('执行步骤：')
                for s in steps[:30]:
                    lines.append(f"- {s}")
                if len(steps) > 30:
                    lines.append(f"- ……还有 {len(steps)-30} 步")
            lines.append('')

    # 6. 底层关键函数解释（来自 user.py）
    lines.append('## 六、底层关键函数解释（真正控制机器人的代码）')
    lines.append('')
    lines.append('上面的积木会被自动转换成 Python 代码。下面是几个最基础、最重要的 Python 函数：')
    lines.append('')

    key_funcs = [
        ('设置', '初始化传感器阈值等参数'),
        ('_getAllADC', '读取 5 路光电传感器数值'),
        ('_read_sensor_error', '根据传感器数值计算机器人偏离黑线的程度'),
        ('_calc_pid', 'PID 算法，算出电机需要修正多少'),
        ('_motor_control', '把 PID 结果变成左右电机速度'),
        ('巡线遇线停', '沿着线走，直到某个传感器检测到线才停'),
        ('巡线时间', '沿着线走指定时间'),
        ('巡线编码', '沿着线走指定编码器距离'),
        ('自定义转弯', '让左右轮以不同速度转，实现转弯'),
        ('走距离cm不循线', '不巡线，直接走指定厘米数'),
        ('走编码不循线', '不巡线，按编码器计数走'),
        ('转弯_速度单位每秒cm数', '按 cm/s 速度转弯'),
        ('设置机器参数', '设置轮胎直径、轮距、码盘线数、减速比'),
        ('抬升夹子B4', '控制机械臂/夹子抬升'),
        ('夹子张开与关闭A4', '控制夹子张开或关闭'),
        ('设置机械臂_升', '升起机械臂'),
        ('设置机械臂_降', '降下机械臂'),
    ]

    for py_name, human_desc in key_funcs:
        if py_name in user_py_groups:
            info = explain_function_code(py_name, user_py_groups[py_name])
            lines.append(f"### `{py_name}`")
            lines.append('')
            lines.append(f"**人话解释**：{human_desc}")
            lines.append('')
            if info:
                lines.append('**涉及动作**：' + '、'.join(info))
            lines.append(f"**代码行数**：{len(user_py_groups[py_name])} 行")
            lines.append('')

    # 7. 零基础概念解释
    lines.append('## 七、零基础必备概念解释')
    lines.append('')
    concepts = [
        ('巡线', '机器人用底部的光电传感器识别地上的黑线/白线，然后沿着线走。'),
        ('光电传感器', '像机器人的"眼睛"，能区分黑和白。黑线反光弱，白色地面反光强。'),
        ('PID', '一种自动修正算法。机器人发现偏离线了，PID 会算出要往哪边调整、调多少。'),
        ('舵机', '能转到指定角度的电机，常用来控制夹子、机械臂、闸门等。'),
        ('编码器', '装在轮子上的"计数器"，机器人通过它知道自己走了多远。'),
        ('码盘/减速比', '和轮子转一圈的计数有关的参数，用来把编码器计数换算成厘米。'),
        ('函数', '把一段常用动作打包起来，起个名字，需要时直接调用。'),
        ('主程序', '机器人开机后第一个开始执行的程序。'),
    ]
    for title, desc in concepts:
        lines.append(f"### {title}")
        lines.append(desc)
        lines.append('')

    # 8. 总结
    lines.append('## 八、总结')
    lines.append('')
    lines.append('这个项目是一个完整的 **XYRobot-S3 自动搬运机器人程序**：')
    lines.append('- 主程序按顺序调用各种任务函数；')
    lines.append('- 底层用 PID 算法实现稳定巡线；')
    lines.append('- 用舵机和机械臂完成「夹、抬、放」等动作；')
    lines.append('- 函数命名很直观（球X杯Y、易拉罐N、泡沫等），对应比赛中的不同得分任务。')
    lines.append('')
    lines.append('如果你是零基础，建议先从 **第六章的底层函数** 开始看，理解机器人怎么「走、转、夹」，再回头看 **第四章主程序** 的任务流程。')
    lines.append('')

    return '\n'.join(lines)


def main():
    data = load_spy()
    doc = build_doc(data)
    with open(OUT_MD, 'w', encoding='utf-8') as f:
        f.write(doc)
    print(f'已生成说明文档：{OUT_MD}')
    print(f'文档字数：{len(doc)}')


if __name__ == '__main__':
    main()
