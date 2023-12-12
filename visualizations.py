import matplotlib.pyplot as plt


def visualize_animation_fun_SineAnimation():
    # read from file
    filename = "./result/animation_fun/sine_animation.txt"
    input_str = ""
    with open(filename) as f:
        input_str = f.read()


    # 将字符串分割成行
    lines = input_str.strip().split("\n")

    # 初始化time和x的列表
    times = []
    xs = []

    # 解析每一行
    for line in lines:
        if line == "":
            continue
        parts = line.split(", ")
        frame, time, x = parts
        times.append(float(time.split(": ")[1]))
        xs.append(float(x.split(": ")[1]))

    # 使用matplotlib绘图
    plt.plot(times, xs)
    plt.xlabel('Time')
    plt.ylabel('X')
    plt.title('Frame Data')
    plt.show()


if __name__ == "__main__":
    visualize_animation_fun_SineAnimation()