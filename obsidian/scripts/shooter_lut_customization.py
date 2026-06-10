import sys
import tkinter as tk
import customtkinter as ctk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import re

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class LUTApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Shooter LUT Customization - Interactive")
        self.geometry("900x600")

        # Initial data
        self.points = [
            {"d": 80.70, "v": 1200.0},
            {"d": 46.69, "v": 1050.0},
            {"d": 118.60, "v": 1400.0},
            {"d": 108.52, "v": 1370.0}
        ]

        self.dragging_idx = None

        # Setup Layout
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # Left Frame for inputs
        self.left_frame = ctk.CTkFrame(self, width=350)
        self.left_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        self.title_label = ctk.CTkLabel(self.left_frame, text="LUT Points", font=ctk.CTkFont(size=20, weight="bold"))
        self.title_label.pack(pady=(10, 5))
        
        self.hint_label = ctk.CTkLabel(self.left_frame, text="(You can also drag points\non the graph!)", text_color="gray")
        self.hint_label.pack(pady=(0, 10))

        self.actions_frame = ctk.CTkFrame(self.left_frame, fg_color="transparent")
        self.actions_frame.pack(fill="x", padx=10, pady=5)
        
        self.paste_btn = ctk.CTkButton(self.actions_frame, text="Paste Code from Clipboard", command=self.paste_code, fg_color="#28a745", hover_color="#218838")
        self.paste_btn.pack(fill="x", pady=5)

        self.entries_frame = ctk.CTkScrollableFrame(self.left_frame, fg_color="transparent")
        self.entries_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.entries = []
        self.string_vars = []
        self.draw_entries()

        # Right Frame for plot and code
        self.right_frame = ctk.CTkFrame(self)
        self.right_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        self.right_frame.grid_rowconfigure(0, weight=3)
        self.right_frame.grid_rowconfigure(1, weight=1)
        self.right_frame.grid_columnconfigure(0, weight=1)

        # Plot
        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.figure.patch.set_facecolor('#2b2b2b')
        self.ax = self.figure.add_subplot(111)
        self.ax.set_facecolor('#2b2b2b')
        self.ax.tick_params(colors='white')
        self.ax.xaxis.label.set_color('white')
        self.ax.yaxis.label.set_color('white')
        self.ax.spines['bottom'].set_color('white')
        self.ax.spines['top'].set_color('white')
        self.ax.spines['left'].set_color('white')
        self.ax.spines['right'].set_color('white')
        self.ax.set_xlabel("Distance")
        self.ax.set_ylabel("Velocity")
        
        self.line, = self.ax.plot([], [], linestyle='-', color='#1f538d', linewidth=2)
        self.scatter, = self.ax.plot([], [], 'o', color='#1f538d', markersize=10, picker=True, pickradius=10)

        self.canvas = FigureCanvasTkAgg(self.figure, self.right_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        # Connect Matplotlib events
        self.canvas.mpl_connect('button_press_event', self.on_press)
        self.canvas.mpl_connect('button_release_event', self.on_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_motion)

        # Code output
        self.code_frame = ctk.CTkFrame(self.right_frame)
        self.code_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        self.code_frame.grid_rowconfigure(0, weight=1)
        self.code_frame.grid_columnconfigure(0, weight=1)

        self.code_textbox = ctk.CTkTextbox(self.code_frame, font=ctk.CTkFont(family="Consolas", size=14))
        self.code_textbox.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        self.copy_btn = ctk.CTkButton(self.code_frame, text="Copy to Clipboard", command=self.copy_code)
        self.copy_btn.grid(row=1, column=0, pady=5)

        self.update_plot_and_code()

    def draw_entries(self):
        for widget in self.entries_frame.winfo_children():
            widget.destroy()
        
        self.entries = []
        self.string_vars = []
        
        # Configure columns so they center nicely
        self.entries_frame.grid_columnconfigure(0, weight=1)
        self.entries_frame.grid_columnconfigure(1, weight=1)
        self.entries_frame.grid_columnconfigure(2, weight=0)

        ctk.CTkLabel(self.entries_frame, text="Distance").grid(row=0, column=0, padx=2, pady=5)
        ctk.CTkLabel(self.entries_frame, text="Velocity").grid(row=0, column=1, padx=2, pady=5)

        for i, pt in enumerate(self.points):
            sv_d = tk.StringVar(value=f"{pt['d']:.2f}")
            sv_v = tk.StringVar(value=f"{pt['v']:.2f}")
            
            sv_d.trace_add("write", lambda *args, idx=i, var=sv_d, key="d": self.on_entry_change(idx, key, var))
            sv_v.trace_add("write", lambda *args, idx=i, var=sv_v, key="v": self.on_entry_change(idx, key, var))

            self.string_vars.append((sv_d, sv_v))

            entry_d = ctk.CTkEntry(self.entries_frame, textvariable=sv_d, width=70)
            entry_d.grid(row=i+1, column=0, padx=2, pady=5)
            
            entry_v = ctk.CTkEntry(self.entries_frame, textvariable=sv_v, width=70)
            entry_v.grid(row=i+1, column=1, padx=2, pady=5)
            
            del_btn = ctk.CTkButton(self.entries_frame, text="X", width=28, fg_color="#dc3545", hover_color="#c82333", command=lambda idx=i: self.remove_point(idx))
            del_btn.grid(row=i+1, column=2, padx=5, pady=5)

            self.entries.append((entry_d, entry_v))

        add_btn = ctk.CTkButton(self.entries_frame, text="+ Add Point", command=self.add_point)
        add_btn.grid(row=len(self.points)+1, column=0, columnspan=3, pady=(15, 5))

    def add_point(self):
        if self.points:
            last = self.points[-1]
            self.points.append({"d": last["d"] + 10, "v": last["v"]})
        else:
            self.points.append({"d": 0.0, "v": 0.0})
        self.draw_entries()
        self.update_plot_and_code()

    def remove_point(self, idx):
        if len(self.points) > 1:
            self.points.pop(idx)
            self.draw_entries()
            self.update_plot_and_code()

    def paste_code(self):
        try:
            clipboard_text = self.clipboard_get()
            # Match ShooterVelocityLut.sample(d, v) or just sample(d, v)
            matches = re.findall(r'sample\s*\(\s*([\d\.]+)\s*,\s*([\d\.]+)\s*\)', clipboard_text)
            if matches:
                new_points = []
                for d_str, v_str in matches:
                    new_points.append({"d": float(d_str), "v": float(v_str)})
                self.points = new_points
                self.draw_entries()
                self.update_plot_and_code()
            else:
                print("No valid 'sample(distance, velocity)' formats found in clipboard.")
        except Exception as e:
            print("Error pasting from clipboard:", e)

    def update_entries_from_points(self):
        for i, pt in enumerate(self.points):
            sv_d, sv_v = self.string_vars[i]
            if sv_d.get() != f"{pt['d']:.2f}":
                sv_d.set(f"{pt['d']:.2f}")
            if sv_v.get() != f"{pt['v']:.2f}":
                sv_v.set(f"{pt['v']:.2f}")

    def on_entry_change(self, idx, key, var):
        if self.dragging_idx is not None:
            return # Don't update from entries while dragging
        try:
            val = float(var.get())
            self.points[idx][key] = val
            self.update_plot_and_code(update_entries=False)
        except ValueError:
            pass
        except IndexError:
            # Indices might temporarily be out of sync during removal
            pass

    def update_plot_and_code(self, update_entries=True):
        if update_entries:
            self.update_entries_from_points()

        # Sort points by distance for drawing the line
        sorted_pts = sorted(self.points, key=lambda x: x["d"])
        line_x = [pt["d"] for pt in sorted_pts]
        line_y = [pt["v"] for pt in sorted_pts]
        
        self.line.set_data(line_x, line_y)

        scat_x = [pt["d"] for pt in self.points]
        scat_y = [pt["v"] for pt in self.points]
        self.scatter.set_data(scat_x, scat_y)

        if self.dragging_idx is None and len(self.points) > 0:
            all_x = scat_x
            all_y = scat_y
            x_range = max(all_x) - min(all_x)
            y_range = max(all_y) - min(all_y)
            margin_x = max(20, x_range * 0.2)
            margin_y = max(100, y_range * 0.2)
            self.ax.set_xlim(min(all_x) - margin_x, max(all_x) + margin_x)
            self.ax.set_ylim(min(all_y) - margin_y, max(all_y) + margin_y)

        self.canvas.draw()

        # Update Code
        code = "    public static final ShooterVelocityLut SHOOTER_VELOCITY_LUT = new ShooterVelocityLut(\n"
        for i, pt in enumerate(self.points):
            comma = "," if i < len(self.points) - 1 else ""
            code += f"        ShooterVelocityLut.sample({pt['d']:.2f}, {pt['v']:.1f}){comma}\n"
        code += "    );"

        self.code_textbox.delete("1.0", tk.END)
        self.code_textbox.insert(tk.END, code)

    def get_closest_point(self, x, y):
        min_dist = float('inf')
        closest_idx = None
        
        # Get axis limits to normalize distance calculation
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        x_range = xlim[1] - xlim[0]
        y_range = ylim[1] - ylim[0]
        
        if x_range == 0 or y_range == 0:
            return None

        for i, pt in enumerate(self.points):
            dx = (pt["d"] - x) / x_range
            dy = (pt["v"] - y) / y_range
            dist = dx**2 + dy**2
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Normalized threshold distance
        if min_dist < 0.01:
            return closest_idx
        return None

    def on_press(self, event):
        if event.inaxes != self.ax:
            return
        idx = self.get_closest_point(event.xdata, event.ydata)
        if idx is not None:
            self.dragging_idx = idx

    def on_release(self, event):
        self.dragging_idx = None
        self.update_plot_and_code()

    def on_motion(self, event):
        if self.dragging_idx is None:
            return
        if event.inaxes != self.ax:
            return
        
        # update point
        self.points[self.dragging_idx]["d"] = event.xdata
        self.points[self.dragging_idx]["v"] = event.ydata
        self.update_plot_and_code()

    def copy_code(self):
        code = self.code_textbox.get("1.0", tk.END).strip()
        self.clipboard_clear()
        self.clipboard_append(code)
        self.update()

if __name__ == "__main__":
    app = LUTApp()
    app.mainloop()