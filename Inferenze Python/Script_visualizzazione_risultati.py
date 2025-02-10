# Valori delle metriche
metrics = ["Accuracy", "Precision", "Recall", "Specifity", "F1-score", "AUROC"]
values = [accuracy, precision, recall, specificity, f1, auroc]

# Creazione del grafico a barre con larghezza 0.3
plt.figure(figsize=(8, 5))
plt.bar(metrics, values, color='skyblue', width=0.3)

# Aggiunta dei valori numerici sopra le barre con 4 cifre decimali
for i, v in enumerate(values):
    plt.text(i, v + 0.02, f"{v:.4f}", ha='center', fontsize=12)

plt.ylim(0, 1.1)  # Per dare un po' di spazio sopra le barre
plt.ylabel("Valore")
plt.title("Metriche di Classificazione (Rete CNN)")
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.show()

# ---- Grafico AUROC ----
fpr, tpr, _ = roc_curve(output_data, predicted_scores)

plt.figure(figsize=(8, 5))
plt.plot(fpr, tpr, color='blue', lw=2, label=f'AUROC = {auroc:.4f}')
plt.plot([0, 1], [0, 1], color='navy', lw=2, linestyle='--')  # Linea diagonale
plt.xlim([0.0, 1.0])
plt.ylim([0.0, 1.05])
plt.xlabel("False Positive Rate")
plt.ylabel("True Positive Rate")
plt.title("Curva ROC (CNN)")
plt.legend(loc="lower right")
plt.grid()
plt.show()
plt.show()

plt.savefig("metriche_classificazione.png", dpi=300)
plt.savefig("curva_roc.png", dpi=300)