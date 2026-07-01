import random
import matplotlib.pyplot as plt

# --- KONSTANTEN ---
NUM_CYCLES = 20000
FILTER_ALPHA = 0.01
MAX_FAILURE_RATE_LIMIT = 0.10  # 10%

# --- SIMULATIONS-PARAMETER ---
TRUE_FAILURE_RATE = 0.03  # 8% echte Fehlerrate

# --- INITIALISIERUNG ---
filtered_failure_rate = 0.0
consecutive_failures = 0

history_filtered_rate = []
history_raw_errors = []  # Speichert 1 für Fehler, 0 für OK
fault_tripped_at = None

# --- SIMULATIONSLOOP ---
for cycle in range(NUM_CYCLES):
    # True = Paket weg (Error), False = Paket da (OK)
    is_packet_lost = random.random() < TRUE_FAILURE_RATE
    
    current_error = 1.0 if is_packet_lost else 0.0
    filtered_failure_rate = (1.0 - FILTER_ALPHA) * filtered_failure_rate + FILTER_ALPHA * current_error
    
    if is_packet_lost:
        consecutive_failures += 1
    else:
        consecutive_failures = 0
        
    if (filtered_failure_rate > MAX_FAILURE_RATE_LIMIT or consecutive_failures >= 5) and fault_tripped_at is None:
        fault_tripped_at = cycle

    history_filtered_rate.append(filtered_failure_rate * 100.0)
    history_raw_errors.append(current_error)

# --- PLOTTEN (Mit zwei Subplots untereinander) ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True, 
                               gridspec_kw={'height_ratios': [3, 1]})

# --- Oberer Plot: Der Filter ---
ax1.plot(history_filtered_rate, label="Gefilterte Ausfallrate (%)", color="blue", linewidth=2)
ax1.axhline(y=MAX_FAILURE_RATE_LIMIT * 100.0, color="red", linestyle="--", label="10% Limit (Notaus)")

if fault_tripped_at is not None:
    ax1.axvline(x=fault_tripped_at, color="purple", linestyle=":", label=f"Notaus bei Zyklus {fault_tripped_at}")
    ax2.axvline(x=fault_tripped_at, color="purple", linestyle=":")

ax1.set_title(f"Simulation des Kommunikationsfilters (Wahre Fehlerrate: {TRUE_FAILURE_RATE*100:.1f}%)")
ax1.set_ylabel("Filter-Fehlerrate (%)")
ax1.grid(True, linestyle=":", alpha=0.6)
ax1.legend(loc="upper right")
ax1.set_ylim(-5, 25) # Angepasst auf 25% für bessere Details im normalen Bereich

# --- Unterer Plot: Die exakten Fehler-Frames ---
# Wir plotten nur die Indizes, bei denen wirklich ein Fehler vorlag (Wert == 1)
error_cycles = [i for i, err in enumerate(history_raw_errors) if err == 1]
ok_cycles = [i for i, err in enumerate(history_raw_errors) if err == 0]

# "Strichliste" zeichnen
ax2.vlines(error_cycles, ymin=0, ymax=1, colors="red", linewidth=1.5, label="Paket VERPASST (Fehler)")
ax2.vlines(ok_cycles, ymin=-1, ymax=0, colors="green", linewidth=0.5, alpha=0.3, label="Paket EMPFANGEN (OK)")

ax2.set_title("Exakte Frame-Historie")
ax2.set_xlabel("Steuerungszyklus (ms)")
ax2.set_yticks([]) # Keine Y-Achsen-Beschriftung nötig, da es eine reine Event-Timeline ist
ax2.set_ylim(-1.2, 1.2)
ax2.grid(True, linestyle=":", alpha=0.3, axis="x")
ax2.legend(loc="upper right")

plt.tight_layout()
plt.show()