import numpy as np
import matplotlib.pyplot as plt
 
# ── Paramètres ────────────────────────────────────────────────────────────────
kn    = 5e4
kt    = 1e4
a     = 0.014
b     = 0.011
e     = 0.021
h     = 0.014  
R     = 0.15
H     = 0.10
omega = 1.0     # rad/s
 

theta0 = np.arctan2(-(h - b), b)
print(f"θ₀ = {np.degrees(theta0):.2f}°  →  arc [{np.degrees(theta0):.1f}°, {np.degrees(np.pi - theta0):.1f}°]")
 

phi_min = np.radians(-45)
phi_max = np.radians( 45)
T       = (phi_max - phi_min) / omega
 
N_steps = 500
t_full  = np.linspace(0, T, N_steps)
phi_arr = phi_min + omega * t_full     # φ(t) linéaire
 

z_centre   = R * np.cos(phi_arr) - H
in_contact = z_centre >= -b
phi_contact = np.degrees(np.arccos((H - b) / R))
print(f"Contact pour |φ| ≤ {phi_contact:.2f}°  ({np.sum(in_contact)} pas sur {N_steps})")
 

# v = dpos/dt = ω·(R cos φ, −R sin φ)
# normalisé : ev = (cos φ, −sin φ)
ev_x =  np.cos(phi_arr)
ev_z = -np.sin(phi_arr)
 
 
N_int = 2000
t_int  = np.linspace(theta0, np.pi - theta0, N_int)
dtheta = t_int[1] - t_int[0]
sin_t  = np.sin(t_int)
cos_t  = np.cos(t_int)
denom  = np.sqrt(a**2 * sin_t**2 + b**2 * cos_t**2)
depth  = np.maximum(h - b * sin_t, 0.0)   # profondeur fixe
 
Fx_out = np.zeros(N_steps)
Fz_out = np.zeros(N_steps)
 
for idx in range(N_steps):
    if not in_contact[idx]:
        continue
 
    evx = ev_x[idx]
    evz = ev_z[idx]
 
    en_dot_ev_raw = b * cos_t * evx + a * sin_t * evz
    et_dot_ev_raw = -a * sin_t * evx + b * cos_t * evz
 
    Fx_out[idx] = e * dtheta * np.sum(depth * (-kn * en_dot_ev_raw * b * cos_t
                                               - kt * et_dot_ev_raw * a * sin_t) / denom)
    Fz_out[idx] = e * dtheta * np.sum(depth * ( kn * en_dot_ev_raw * a * sin_t
                                               - kt * et_dot_ev_raw * b * cos_t) / denom)
 
 
dt  = t_full[1] - t_full[0]
vx  =  R * np.cos(phi_arr) * omega
vz  = -R * np.sin(phi_arr) * omega
dW      = (Fx_out * vx + Fz_out * vz) * dt
W_cumul = np.cumsum(dW)
W_total = W_cumul[-1]
print(f"Travail total W = {W_total*1e3:.4f} mJ")
 
# Stats
idx_mid = N_steps // 2
print(f"Fx : min={Fx_out[in_contact].min():.3f} N,  max={Fx_out[in_contact].max():.3f} N,  à φ=0° : {Fx_out[idx_mid]:.3f} N")
print(f"Fz : min={Fz_out[in_contact].min():.3f} N,  max={Fz_out[in_contact].max():.3f} N,  à φ=0° : {Fz_out[idx_mid]:.3f} N")
 
 
t_contact = t_full[in_contact]
fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
 
for ax in axes:
    ax.axvspan(t_contact[0], t_contact[-1], alpha=0.08, color='steelblue', label='contact')
 
axes[0].plot(t_full, Fx_out, color='steelblue', label='$F_x$', lw=2)
axes[0].axhline(0, color='gray', lw=0.5, ls='--')
axes[0].set_ylabel("$F_x$ (N)"); axes[0].legend(); axes[0].grid(True, alpha=0.3)
 
axes[1].plot(t_full, Fz_out, color='coral', label='$F_z$', lw=2)
axes[1].axhline(0, color='gray', lw=0.5, ls='--')
axes[1].set_ylabel("$F_z$ (N)"); axes[1].legend(); axes[1].grid(True, alpha=0.3)
 
axes[2].plot(t_full, W_cumul * 1e3, color='seagreen', label='Travail cumulé')
axes[2].axhline(0, color='gray', lw=0.5, ls='--')
axes[2].set_ylabel("Travail (mJ)")
axes[2].set_xlabel(f"Temps (s)  [ω = {omega} rad/s]")
axes[2].legend(); axes[2].grid(True, alpha=0.3)
 
ax2 = axes[0].twinx()
ax2.plot(t_full, np.degrees(phi_arr), color='purple', lw=1, ls='--', alpha=0.5, label='φ(t)')
ax2.set_ylabel("φ (°)", color='purple', alpha=0.7)
ax2.tick_params(axis='y', labelcolor='purple')
ax2.legend(loc='upper right')
 
plt.tight_layout()
plt.show()