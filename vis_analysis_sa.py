import pickle
import numpy as np
import matplotlib.pyplot as plt

def plot_motor_timeseries(
    positions,           # ndarray, shape = (T, N) or (N, T)
    forces,              # ndarray, shape = (T, N) or (N, T)
    dt=0.01,             # 샘플 간격 [초], time=None이면 사용
    time=None,           # 시간 벡터 (길이 T). 제공 시 dt 대신 사용
    time_axis=0,         # 0이면 (T, N), 1이면 (N, T)
    motor_names=None,    # 모터 이름 리스트 길이 N. None이면 M0..M{N-1}
    per_motor=False,     # True면 모터별 개별 그림으로 출력
    show=True,           # 즉시 plt.show()
    save_prefix=None     # 파일 저장 prefix, 예: "run1" -> run1_positions.png 등
):
    """positions/forces를 시간에 따라 2D plot으로 그립니다.
    - positions, forces: 2D ndarray
    - 둘의 shape은 같아야 하며, time_axis로 (T,N)/(N,T) 지정
    - time이 주어지면 dt 대신 time을 사용합니다.
    """
    P = np.asarray(positions)
    F = np.asarray(forces)

    if P.ndim != 2 or F.ndim != 2:
        raise ValueError("positions와 forces는 모두 2차원 배열이어야 합니다.")

    if time_axis == 1:
        P = P.T
        F = F.T
    elif time_axis != 0:
        raise ValueError("time_axis는 0 또는 1만 가능합니다.")

    if P.shape != F.shape:
        raise ValueError(f"positions{P.shape}와 forces{F.shape}의 shape이 일치해야 합니다.")

    T, N = P.shape

    # 시간 벡터 구성
    if time is None:
        t = np.arange(T) * float(dt)
    else:
        t = np.asarray(time)
        if t.ndim != 1 or t.shape[0] != T:
            raise ValueError("time은 길이 T의 1차원 배열이어야 합니다.")

    # 모터 이름
    if motor_names is None:
        motor_names = [f"M{i}" for i in range(N)]
    if len(motor_names) != N:
        raise ValueError("motor_names 길이는 모터 수 N과 같아야 합니다.")

    # ===== 그리기 =====
    if per_motor:
        # 모터별로 '포지션', '포스' 각각 개별 그림 생성 (subplot 사용 안 함)
        for j in range(N):
            fig1 = plt.figure()
            plt.plot(t, P[:, j], label=f"{motor_names[j]} position")
            plt.xlabel("Time [s]")
            plt.ylabel("Position")
            plt.title(f"Position — {motor_names[j]}")
            plt.legend()
            plt.grid(True)
            if save_prefix:
                fig1.savefig(f"{save_prefix}_pos_{motor_names[j]}.png", dpi=150, bbox_inches="tight")

            fig2 = plt.figure()
            plt.plot(t, F[:, j], label=f"{motor_names[j]} force")
            plt.xlabel("Time [s]")
            plt.ylabel("Force")
            plt.title(f"Force — {motor_names[j]}")
            plt.legend()
            plt.grid(True)
            if save_prefix:
                fig2.savefig(f"{save_prefix}_force_{motor_names[j]}.png", dpi=150, bbox_inches="tight")
    else:
        # 모든 모터를 한 그림에 겹쳐서 (positions), 다른 한 그림에 (forces)
        fig1 = plt.figure()
        for j in range(N):
            plt.plot(t, P[:, j], label=motor_names[j])
        plt.xlabel("Time [s]")
        plt.ylabel("Position")
        plt.title("Motor Positions vs Time")
        # 모터가 많다면 전부 표시가 부담될 수 있음 → 필요시 주석 처리
        plt.legend(ncols=2, fontsize="small")
        plt.grid(True)
        if save_prefix:
            fig1.savefig(f"./data/{save_prefix}_positions.png", dpi=150, bbox_inches="tight")

        fig2 = plt.figure()
        for j in range(N):
            plt.plot(t, F[:, j], label=motor_names[j])
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity")
        plt.title("Motor Velocity vs Time")
        plt.legend(ncols=2, fontsize="small")
        plt.grid(True)
        if save_prefix:
            fig2.savefig(f"./data/{save_prefix}_velocity.png", dpi=150, bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close('all')


# -------------------------
# 사용 예시 (더미 데이터)
# -------------------------
if __name__ == "__main__":
    T = 1600         # 10초 @ 0.01s
    N = 7            # 모터 4개
    dt = 0.01
    t = np.arange(T) * dt
    
    f = open("./data/state_action_data.pkl", "rb")
    sa = pickle.load(f)

    franka1_state = np.concatenate([ v.cpu().numpy().reshape(1, -1) for v in sa["franka1_state"]], axis=0)
    franka1_action = np.concatenate([ v.cpu().numpy().reshape(1, -1) for v in sa["franka1_action"]], axis=0)
    franka2_state = np.concatenate([ v.cpu().numpy().reshape(1, -1) for v in sa["franka2_state"]], axis=0)
    franka2_action = np.concatenate([ v.cpu().numpy().reshape(1, -1) for v in sa["franka2_action"]], axis=0)

    print(franka1_state.shape)
    print(franka1_action.shape)
    print(franka2_state.shape)
    print(franka2_action.shape)

    franka2_action[0, :] = 0
    franka1_action[0, :] = 0

    plot_motor_timeseries(
        franka1_state, franka1_action,
        dt=dt,
        time=None,           # 또는 time=t
        time_axis=0,         # positions/forces가 (T,N)일 때
        motor_names=[f"J{i+1}" for i in range(N)],
        per_motor=False,     # True로 바꾸면 모터별 개별 그림 출력
        show=False,
        save_prefix="franka_1"     # "run1" 등으로 넣으면 PNG 저장
    )
    
    plot_motor_timeseries(
        franka2_state, franka2_action,
        dt=dt,
        time=None,           # 또는 time=t
        time_axis=0,         # positions/forces가 (T,N)일 때
        motor_names=[f"J{i+1}" for i in range(N)],
        per_motor=False,     # True로 바꾸면 모터별 개별 그림 출력
        show=False,
        save_prefix="franka_2"     # "run1" 등으로 넣으면 PNG 저장
    )



