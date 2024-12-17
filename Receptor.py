import cv2
import numpy as np
import time
import pandas as pd


def recognize_bits(bit_region, threshold=127):
    gray_region = cv2.cvtColor(bit_region, cv2.COLOR_BGR2GRAY)
    mean_intensity = np.mean(gray_region)
    return 1 if mean_intensity > threshold else 0


def is_color_similar(region, color="green"):
    # Reduzir o ruído
    region = cv2.GaussianBlur(region, (5, 5), 0)
    hsv_region = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

    if color == "green":
        lower_bound = np.array([40, 50, 50])
        upper_bound = np.array([80, 255, 255])
        # Máscara para detectar a cor verde
        mask = cv2.inRange(hsv_region, lower_bound, upper_bound)

        blue_pixels = cv2.countNonZero(mask)
        return blue_pixels > (bit_region.size // 4)

    elif color == "red":
        # Definir os limites da cor vermelha no espaço de cor HSV
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 500, 500])
        upper_red2 = np.array([180, 255, 255])

        # Máscara para detectar a cor vermelha
        mask1 = cv2.inRange(hsv_region, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_region, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        red_pixels = cv2.countNonZero(mask)
        return red_pixels > (bit_region.size // 8)  # Proporção de pixels vermelhos


def compare_strings(string1, string2):
    smaller_length = min(len(string1), len(string2))

    total_matches = sum(1 for i in range(smaller_length) if string1[i] == string2[i])

    return total_matches


# cap = cv2.VideoCapture("https://192.168.3.16:8080/video")
cap = cv2.VideoCapture("https://192.168.3.17:8080/video")

res_original = (1920, 1080)
res_nova = (1280, 720)

scale_w = res_nova[0] / res_original[0]
scale_h = res_nova[1] / res_original[1]

bit_x, bit_y = int(400 * scale_h), int(1000 * scale_w)
timing_x, timing_y = int(400 * scale_h), int(900 * scale_w)
bit_size = int(90 * scale_h)
timing_size = int(90 * scale_w)

string_captured = ""
last_timing_bit = None
can_start_capture = False


frame_counter = 0  # Contar os frames
cont = 0

green = (0, 255, 0)
red = (0, 0, 255)

# Armazenar os bits para fazer a média
bit_values = []
collecting_bits = False  # Indicar se está coletando o bit após a mudança detectada.
bit_frame_counter = 0

correct_sequence = "101010101010"
df_bits_collected = pd.DataFrame(
    columns=[
        "Sequência Coletada",
        "Sequência Correta de Bits",
        "Bits Acertados",
        "Mais Bits Que o Correto",
    ]
)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Falha na captura da webcam.")
        break

    bit_region = frame[bit_x : bit_x + bit_size, bit_y : bit_y + bit_size]
    timing_bit_region = frame[
        timing_x : timing_x + timing_size, timing_y : timing_y + timing_size
    ]

    cv2.rectangle(frame, (bit_y, bit_x), (bit_y + bit_size, bit_x + bit_size), green, 2)
    cv2.rectangle(
        frame,
        (timing_y, timing_x),
        (timing_y + timing_size, timing_x + timing_size),
        red,
        2,
    )

    if frame_counter >= 70:
        if is_color_similar(bit_region, "green"):
            can_start_capture = True
            bit_value = "Green Signal Detected"
            frame_counter = 0
        elif is_color_similar(bit_region, "red"):
            if can_start_capture:
                df_bits_collected = df_bits_collected.append(
                    {
                        "Sequência Coletada": string_captured,
                        "Sequência Correta de Bits": correct_sequence,
                        "Bits Acertados": compare_strings(
                            string_captured, correct_sequence
                        ),
                        "Mais Bits que o Correto": len(string_captured)
                        > len(correct_sequence),
                    },
                    ignore_index=True,
                )

                print(f"String Captured: {string_captured}")
                string_captured = ""
            can_start_capture = False
            frame_counter = 0
        timing_bit = None

        if (
            can_start_capture
            and not is_color_similar(bit_region, "green")
            and not is_color_similar(bit_region, "red")
        ):

            timing_bit = recognize_bits(timing_bit_region)  #
            if last_timing_bit != timing_bit:
                time.sleep(0.1)

                bit_value = recognize_bits(bit_region)
                string_captured += str(bit_value)
                last_timing_bit = timing_bit
                frame_counter = 0

                frame_filename = f"frame_{cont}.jpg"
                cv2.imwrite(frame_filename, frame)
                cont += 1
        else:
            bit_value = "Waiting for Green Signal"

        cv2.putText(
            frame,
            f"Bit Value: {bit_value}",
            (50, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            green,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"Timing Bit: {timing_bit if can_start_capture else 'N/A'}",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            red,
            2,
            cv2.LINE_AA,
        )

    cv2.imshow("Reconhecimento de Bits", frame)

    frame_counter += 1

    # Controle de saída e ajustes manuais
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("j"):
        bit_y -= 5
    elif key == ord("l"):
        bit_y += 5
    elif key == ord("i"):
        bit_x -= 5
    elif key == ord("k"):
        bit_x += 5
    elif key == ord("a"):
        timing_y -= 5
    elif key == ord("d"):
        timing_y += 5
    elif key == ord("w"):
        timing_x -= 5
    elif key == ord("s"):
        timing_x += 5
    elif key == ord("u"):
        bit_size += 5
    elif key == ord("o"):
        bit_size = max(5, bit_size - 5)
    elif key == ord("y"):
        timing_size += 5
    elif key == ord("h"):
        timing_size = max(5, timing_size - 5)

print(
    f"Coordenadas e Tamanho do Bit Region: Posição ({bit_x}, {bit_y}), Tamanho {bit_size}"
)
print(
    f"Coordenadas e Tamanho do Timing Bit Region: Posição ({timing_x}, {timing_y}), Tamanho {timing_size}"
)

df_bits_collected.to_excel("bits_collected.xlsx", index=False)

cap.release()
cv2.destroyAllWindows()