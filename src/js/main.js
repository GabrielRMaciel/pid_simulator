import { PIDController } from './pid-controller.js';
import { MechanicalSystem } from './plant-models.js';

class SimulationApp {
    constructor() {
        this.ui = {
            kpSlider: document.getElementById('kp-slider'),
            kiSlider: document.getElementById('ki-slider'),
            kdSlider: document.getElementById('kd-slider'),
            kpValue: document.getElementById('kp-value'),
            kiValue: document.getElementById('ki-value'),
            kdValue: document.getElementById('kd-value'),
            resetButton: document.getElementById('reset-button'),
            disturbanceButton: document.getElementById('disturbance-button'),
            presetButtons: document.querySelectorAll('.preset-button'),
            pidChartCanvas: document.getElementById('pidChart'),
            currentErrorSpan: document.getElementById('currentErrorSpan'),
            pTermSpan: document.getElementById('pTermSpan'),
            iTermSpan: document.getElementById('iTermSpan'),
            dTermSpan: document.getElementById('dTermSpan'),
        };
        
        this.config = { SIMULATION_TIMESTEP_S: 0.04, MAX_DATA_POINTS: 300, DISTURBANCE_FORCE: -30 };
        this.presets = {
            p_only: { kp: 1.0, ki: 0, kd: 0 },
            pi: { kp: 1.2, ki: 0.4, kd: 0 },
            pid_damped: { kp: 1.5, ki: 0.5, kd: 3.5 },
            pid_oscillatory: { kp: 8.0, ki: 2.0, kd: 1.0 }
        };

        this.plant = new MechanicalSystem({ inertia: 1.0, friction: 0.2, load: 20.0 });
        this.pid = new PIDController(this.ui.kpSlider.value, this.ui.kiSlider.value, this.ui.kdSlider.value, { outputMin: -100, outputMax: 100 });
        this.setpoint = 80;
        this.isRunning = false; this.timeAccumulator = 0; this.lastFrameTime = null;

        this.initializeChart();
        this.setupControlListeners();
        this.resetSimulation();
    }

    initializeChart() {
        const ctx = this.ui.pidChartCanvas.getContext('2d');
        this.chart = new Chart(ctx, {
            type: 'line', data: { labels: [], datasets: [
                { label: 'Setpoint (SP)', data: [], borderColor: 'green', borderWidth: 2, pointRadius: 0 },
                { label: 'Processo (PV)', data: [], borderColor: 'blue', borderWidth: 2, pointRadius: 0 },
                { label: 'Saída (MV %)', data: [], borderColor: 'purple', borderWidth: 1.5, yAxisID: 'y1', pointRadius: 0 }
            ]},
            options: { responsive: true, maintainAspectRatio: false, animation: false,
                scales: {
                    y: { min: -10, max: 120, title: { display: true, text: 'Valor' } },
                    y1: { type: 'linear', position: 'right', min: -110, max: 110, title: { display: true, text: 'Saída (%)' }, grid: { drawOnChartArea: false } }
                }
            }
        });
    }

    setupControlListeners() {
        const updateFromSliders = () => {
            this.ui.kpValue.textContent = this.ui.kpSlider.value;
            this.ui.kiValue.textContent = this.ui.kiSlider.value;
            this.ui.kdValue.textContent = this.ui.kdSlider.value;
            this.pid.setGains(this.ui.kpSlider.value, this.ui.kiSlider.value, this.ui.kdSlider.value);
        };
        this.ui.kpSlider.addEventListener('input', updateFromSliders);
        this.ui.kiSlider.addEventListener('input', updateFromSliders);
        this.ui.kdSlider.addEventListener('input', updateFromSliders);
        this.ui.resetButton.addEventListener('click', () => this.resetSimulation());
        this.ui.disturbanceButton.addEventListener('click', () => this.toggleDisturbance());
        this.ui.presetButtons.forEach(button => {
            button.addEventListener('click', () => this.applyPreset(button.dataset.preset));
        });
    }

    resetSimulation() {
        this.stop(); this.plant.reset(); this.pid.reset();
        this.ui.disturbanceButton.classList.remove('active');
        this.chart.data.labels = [];
        this.chart.data.datasets.forEach(d => d.data = []);
        this.start();
    }
    
    applyPreset(presetName) {
        const preset = this.presets[presetName];
        if (!preset) return;
        this.ui.kpSlider.value = preset.kp;
        this.ui.kiSlider.value = preset.ki;
        this.ui.kdSlider.value = preset.kd;
        this.ui.kpSlider.dispatchEvent(new Event('input'));
        this.ui.kiSlider.dispatchEvent(new Event('input'));
        this.ui.kdSlider.dispatchEvent(new Event('input'));
        this.resetSimulation();
    }

    toggleDisturbance() {
        const isActive = this.plant._disturbance !== 0;
        this.plant.setDisturbance(isActive ? 0 : this.config.DISTURBANCE_FORCE);
        this.ui.disturbanceButton.classList.toggle('active', !isActive);
    }
    
    simulationLoop = (currentTime) => {
        if (!this.lastFrameTime) this.lastFrameTime = currentTime;
        const deltaTime = (currentTime - this.lastFrameTime) / 1000;
        this.lastFrameTime = currentTime;
        this.timeAccumulator += deltaTime;
        while (this.timeAccumulator >= this.config.SIMULATION_TIMESTEP_S) {
            const pv = this.plant.state.position;
            const mv = this.pid.update(this.setpoint, pv, this.config.SIMULATION_TIMESTEP_S);
            this.plant.update(mv, this.config.SIMULATION_TIMESTEP_S);
            this.updateUI(pv, mv);
            this.timeAccumulator -= this.config.SIMULATION_TIMESTEP_S;
        }
        if (this.isRunning) requestAnimationFrame(this.simulationLoop);
    }

    updateUI(pv, mv) {
        const error = this.setpoint - pv;
        this.ui.currentErrorSpan.textContent = error.toFixed(2);
        this.ui.pTermSpan.textContent = this.pid._proportionalTerm.toFixed(2);
        this.ui.iTermSpan.textContent = this.pid._integralTerm.toFixed(2);
        this.ui.dTermSpan.textContent = this.pid._derivativeTerm.toFixed(2);

        const data = this.chart.data;
        const time = (data.labels.length * this.config.SIMULATION_TIMESTEP_S).toFixed(1);
        data.labels.push(time);
        data.datasets[0].data.push(this.setpoint);
        data.datasets[1].data.push(pv);
        data.datasets[2].data.push(mv);
        if (data.labels.length > this.config.MAX_DATA_POINTS) {
            data.labels.shift();
            data.datasets.forEach(d => d.data.shift());
        }
        this.chart.update();
    }

    start() {
        if (this.isRunning) return; this.isRunning = true; this.lastFrameTime = null;
        requestAnimationFrame(this.simulationLoop);
    }

    stop() { this.isRunning = false; }
}

window.addEventListener('DOMContentLoaded', () => { new SimulationApp(); });