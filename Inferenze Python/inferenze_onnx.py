import onnxruntime as ort
import numpy as np
import time
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix, accuracy_score, precision_score, recall_score, f1_score, classification_report, roc_curve, roc_auc_score
from scipy.special import softmax

model_path = "my_model_MLP.onnx"
# model path = "my_model_CNN.onnx"

try:
    session = ort.InferenceSession(model_path)
    print("Modello ONNX caricato correttamente!")
except Exception as e:
    print(f"Errore nel caricamento del modello ONNX: {e}")
    exit()

# Carico i dati di input e output (riferimenti)
input_data = np.load("total_batch_input.npy").astype(np.float32)
output_data = np.load("total_batch_output.npy").astype(np.int32)

# Verifico le dimensioni dei dati
print("\nInput shape:", input_data.shape)
print("Output shape:", output_data.shape)

# Lista per memorizzare i tempi di inferenza
inference_times = []

# Lista per memorizzare le predizioni
predicted_labels = []
predicted_scores = []

# Ciclo per eseguire le inferenze
for i in range(input_data.shape[0]):

    input_tensor = input_data[i].reshape(1, 4)
    start_time = time.time()
    output = session.run(None, {'actual_input': input_tensor})
    probabilities = softmax(output[0], axis=1)
    end_time = time.time()

    inference_times.append(end_time - start_time)

    # Salva le probabilità dell'output per la ROC Curve
    predicted_scores.append(probabilities[:, 1][0])

    # Ottieni la label predetta
    predicted_label = np.argmax(probabilities, axis=1)[0]
    predicted_labels.append(predicted_label)

# Converto i risultati in numpy array
predicted_labels = np.array(predicted_labels)
predicted_scores = np.array(predicted_scores)

# Calcolo delle metriche
confusion_mat = confusion_matrix(output_data, predicted_labels)
TN, FP, FN, TP = confusion_mat.ravel()
accuracy = accuracy_score(output_data, predicted_labels)
precision = precision_score(output_data, predicted_labels, pos_label=1)  # Precision per la classe guasto (1)
recall = recall_score(output_data, predicted_labels, pos_label=1)  # Recall per la classe guasto (1)
specificity = TN / (TN + FP)
f1 = f1_score(output_data, predicted_labels, pos_label=1)  # F1-score per la classe guasto (1)
auroc = roc_auc_score(output_data, predicted_scores)  # Uso le probabilità per l'AUROC

# Stampa delle metriche
print("\nConfusion matrix:\n", confusion_mat)
print("\nAccuracy:", accuracy)
print("Precision:", precision)
print("Recall:", recall)
print("Specificity:", specificity)
print("F1-score:", f1)
print("Classification Report:\n", classification_report(output_data, predicted_labels))
print("AUROC:", auroc)

print(f"\nInferenze totali: {len(inference_times)}")
print(f"Tempo totale per tutte le inferenze: {sum(inference_times):.6f} seconds")
print("Average inference time:", np.mean(inference_times) * 1e6, "microseconds")