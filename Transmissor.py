import cv2
import numpy as np
import time 

def recognize_bits(frame, bit_region):
    # Converter a região do bit para escala de cinza
    gray_region = cv2.cvtColor(bit_region, cv2.COLOR_BGR2GRAY)
    
    # Calcular a média de brilho na região
    mean_intensity = np.mean(gray_region)
    
    # Definir um limiar para diferenciar preto (0) e branco (1)
    threshold = 127  # Típico valor para diferenciar preto e branco
    
    if mean_intensity > threshold:
        return 1  # Branco
    else:
        return 0  # Preto

# Inicializar a webcam
cap = cv2.VideoCapture(0)

bit_sequence =[]

while True:
    # Capturar o frame da webcam
    ret, frame = cap.read()
    
    if not ret:
        print("Falha na captura da webcam.")
        break
    
    # Exemplo de seleção de uma região onde os bits estão localizados
    # Definir uma área específica para análise (x, y, largura, altura)
    bit_region = frame[100:400, 100:400]
    
    # Mostrar a área do bit
    cv2.rectangle(frame, (100, 100), (400, 400), (0, 255, 0), 2)
    
    # Reconhecer o bit
    bit_value = recognize_bits(frame, bit_region)
    
    # Exibir o valor reconhecido
    cv2.putText(frame, f'Bit: {bit_value}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
    
    # Exibir o frame com o resultado
    cv2.imshow("Reconhecimento de Bits", frame)

    bit_sequence.append(bit_value)
    
    # Parar com a tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    time.sleep(1)

print (bit_sequence)

# Liberar os recursos
cap.release()
cv2.destroyAllWindows()