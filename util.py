from enum import Enum
import math
import environment
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

class EnvType (Enum):
    RURAL = 0
    SUBURBAN = 1
    URBAN = 2
    INDOOR_FR2 = 3
    INDOOR_FR1 = 4


MIN_RSRP = -120 # -140 #dB

def compute_rsrp(ue, bs, env):
    if bs.bs_type == "sat":
        return bs.sat_eirp - bs.path_loss - bs.atm_loss - bs.ut_G_T
    elif bs.bs_type == "drone_relay":
        return bs.compute_rsrp_drone(ue)
    else:
        #lte and nr case
        path_loss = compute_path_loss_cost_hata(ue, bs, env)
        subcarrier_power = 0
        if (bs.bs_type == "lte"):
            subcarrier_power = 10*math.log10(bs.antenna_power*1000 / ((bs.total_prb/10)*bs.number_subcarriers))
        else:
            subcarrier_power = 10*math.log10(bs.antenna_power*1000 / ((bs.total_prb/(10*2**bs.numerology))*bs.number_subcarriers))
        return subcarrier_power + bs.antenna_gain - bs.feeder_loss - path_loss

import math

def compute_path_loss(ue, bs, env, save=None):
    # 计算两点之间的距离（单位：米）
    dist_m = math.sqrt((ue.current_position[0] - bs.position[0])**2 +
                       (ue.current_position[1] - bs.position[1])**2)
    freq_ghz = bs.carrier_frequency / 1000  # 将频率从MHz转换为GHz

    # 判断LOS/NLOS状态，这里简化为总是LOS，实际应用中需要根据环境确定
    is_los = True

    # 根据环境类型选择计算路径损耗的方法
    if env in [EnvType.RURAL, EnvType.SUBURBAN, EnvType.URBAN]:
        # 这里应该根据COST-HATA模型计算路径损耗，暂时省略详细计算
        path_loss = 0  # 应替换为COST-HATA模型的计算结果
    elif env == EnvType.INDOOR_FR2:
        # 使用3GPP mmWave室内模型计算路径损耗
        path_loss = compute_path_loss_3gpp_mmwave(dist_m, freq_ghz, is_los, ue.h_m)
    elif env == EnvType.INDOOR_FR1:
        # 使用简化的FR1室内模型计算路径损耗
        path_loss = 30 + 22 * math.log10(dist_m / 1000) + 20 * math.log10(freq_ghz)  # 注意：这里距离单位是千米
    else:
        raise ValueError("Unsupported environment type.")

    # 如果提供了save参数，保存路径损耗值
    if save is not None:
        save['path_loss'] = path_loss

    return path_loss

# 3GPP mmWave室内模型计算公式
def compute_path_loss_3gpp_mmwave(dist_m, freq_ghz, is_los=True, ut_height=1.5):
    """
    Compute the path loss using 3GPP mmWave indoor model.
    """
    if dist_m == 0:  # Avoid log(0)
        dist_m = 0.01
    
    if is_los:
        # Direct path loss for LOS
        path_loss = 32.4 + 20 * math.log10(dist_m) + 20 * math.log10(freq_ghz)
    else:
        # Path loss for NLOS
        path_loss = 13.54 + 39.08 * math.log10(dist_m) + 20 * math.log10(freq_ghz) - 0.6 * (ut_height - 1.5)
    
    return path_loss

def compute_path_loss_cost_hata(ue, bs, env, save = None):
    #compute distance first
    dist = math.sqrt((ue.current_position[0]-bs.position[0])**2 + (ue.current_position[1]-bs.position[1])**2 + (ue.h_m - bs.h_b)**2)
    if dist == 0:   #just to avoid log(0) in path loss computing
        dist = 0.01
    #compute C_0, C_f, b(h_b), a(h_m) and C_m with the magic numbers defined by the model
    if bs.carrier_frequency <= 1500 and bs.carrier_frequency >= 150 :
        C_0 = 69.55
        C_f = 26.16
        b = 13.82*math.log10(bs.h_b)
        if env.env_type == EnvType.URBAN:
            C_m = 0
        elif env.env_type == EnvType.SUBURBAN:
            C_m = -2*((math.log10(bs.carrier_frequency/28))**2) - 5.4
        else:
            C_m = -4.78*((math.log10(bs.carrier_frequency))**2) + 18.33*math.log10(bs.carrier_frequency) - 40.94
    else:  
        C_0 = 46.3
        C_f = 26.16
        b = 13.82*math.log10(bs.h_b)
        if env.env_type == EnvType.URBAN:
            C_m = 3
        elif env.env_type == EnvType.SUBURBAN:
            C_m = 0
        else:
            raise Exception("COST-HATA model is not defined for frequencies in 1500-2000MHz with RURAL environments")
    
    if env.env_type == EnvType.SUBURBAN or env.env_type == EnvType.RURAL:
        a = (1.1*math.log10(bs.carrier_frequency) - 0.7)*ue.h_m - 1.56*math.log10(bs.carrier_frequency) + 0.8
    else:
        if bs.carrier_frequency >= 150 and bs.carrier_frequency <= 300:
            a = 8.29*(math.log10(1.54*ue.h_m)**2) - 1.1
        else:
            a = 3.2*(math.log10(11.75*ue.h_m)**2) - 4.97
    
    path_loss = C_0 + C_f * math.log10(bs.carrier_frequency) - b - a + (44.9-6.55*math.log10(bs.h_b))*math.log10(dist/1000) + C_m
    if (save is not None):
        save = path_loss
    return path_loss

def find_bs_by_id(bs_id):
    return environment.wireless_environment.bs_list[bs_id]

def find_ue_by_id(ue_id):
    return environment.wireless_environment.ue_list[ue_id]


run = 0


def plot(ue, bs, env):
    global ax
    global fig
    global run
    # if run == 0:
    plt.ion()
    fig, ax = plt.subplots()
    # run = 1

    
    x_ue = []
    y_ue = []
    x_bs = []
    y_bs = []

    plt.cla()
    print(bs)
    #ax.set_xlim(0, env.x_limit)
    #ax.set_ylim(0, env.y_limit)

    colors = cm.rainbow(np.linspace(0, 1, len(bs)))

    for j in bs:
        x_bs.append(find_bs_by_id(j).position[0])
        y_bs.append(find_bs_by_id(j).position[1])

    for i in range(0, len(ue)):
        x_ue.append(find_ue_by_id(ue[i]).current_position[0])
        y_ue.append(find_ue_by_id(ue[i]).current_position[1])

    for i in range(0, len(ue)):
        for j in range(0, len(bs)):
            if find_ue_by_id(ue[i]).current_bs == j:
                ax.scatter(x_ue[i], y_ue[i], color = colors[j])
                break
        else:
            ax.scatter(x_ue[i], y_ue[i], color = "tab:grey")

    for i in range(0, len(ue)):
        ax.annotate(str(ue[i]), (x_ue[i], y_ue[i]))

    for j in range(0, len(bs)):
        if find_bs_by_id(j).bs_type == "drone_relay":
            ax.scatter(x_bs[j], y_bs[j], color = colors[j], label = "BS", marker = "^", s = 400, edgecolor = colors[find_bs_by_id(j).linked_bs], linewidth = 3)
        elif find_bs_by_id(j).bs_type == "drone_bs":
            ax.scatter(x_bs[j], y_bs[j], color = colors[j], label = "BS", marker = "^", s = 400)
        else:
            ax.scatter(x_bs[j], y_bs[j], color = colors[j], label = "BS", marker = "s", s = 400)
    
    for j in range(0, len(bs)):
        ax.annotate("BS"+str(j), (x_bs[j], y_bs[j]))

    ax.grid(True)
    ax.set_ylabel("[m]")
    ax.set_xlabel("[m]")
    fig.canvas.draw()
