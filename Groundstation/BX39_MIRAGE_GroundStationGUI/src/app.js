(function () {
  "use strict";

  const MAX_SAMPLES = 120;
  const TELEMETRY_PERIOD_MS = 1000;
  const EXPECTED_PACKET_SIZE = 217;

  const RELAY_LINES = [
    { id: "relay1", label: "PDB relay 1", pin: "GPIO48 / PDB pin 1" },
    { id: "relay2", label: "PDB relay 2", pin: "GPIO1 / PDB pin 4" },
    { id: "relay3", label: "PDB relay 3", pin: "GPIO2 / PDB pin 5" },
    { id: "relay4", label: "PDB relay 4", pin: "GPIO21 / PDB pin 28" }
  ];

  const PRESSURE_PERIPHERALS = [
    { id: "pump1", label: "vacuum pump 1", offLabel: "OFF", onLabel: "ON" },
    { id: "pump2", label: "vacuum pump 2", offLabel: "OFF", onLabel: "ON" },
    { id: "compressor", label: "compressor", offLabel: "OFF", onLabel: "ON" },
    { id: "outletValve", label: "outlet valve", offLabel: "CLOSED", onLabel: "OPEN" },
    { id: "k96", label: "K96", offLabel: "OFF", onLabel: "ON" }
  ];

  const THERMAL_CHANNELS = [
    { bit: 0, label: "SD-card heater" },
    { bit: 1, label: "Peltier chamber cooler" },
    { bit: 2, label: "outlet heater" },
    { bit: 3, label: "intake preheater" },
    { bit: 4, label: "secondary inlet heater" },
    { bit: 5, label: "Peltier stage 1" },
    { bit: 6, label: "Peltier stage 2" },
    { bit: 7, label: "backup thermal channel" }
  ];

  const DEFAULT_HEATER_MASK = 0x0d;
  const DEFAULT_COOLER_MASK = 0x62;
  const MOCK_DATA_ENABLED = false;
  const HEATER_COMMAND_GUIDE = [
    "heater 1 mode pid target 50",
    "heater 3 mode manual duty 75",
    "HEATER ALL ON"
  ];

  const COMMANDS = {
    startExperiment: {
      label: "start experiment",
      wireCommand: "MODE MEASUREMENTS",
      aliases: ["start experiment", "start measurements", "measurements", "start"],
      effect: function (sim) {
        sim.mode = "MEASUREMENTS";
        sim.pressurisationActive = true;
        sim.setPressureTrain(true);
        sim.outletValveOpen = false;
        sim.setPeripheral("outletValve", false);
        sim.emergencyStopped = false;
        return "autonomous Measurements loop requested";
      }
    },
    enterStandby: {
      label: "enter standby",
      wireCommand: "MODE STANDBY",
      aliases: ["enter standby", "standby", "hold"],
      effect: function (sim) {
        sim.mode = "STANDBY";
        sim.pressurisationActive = false;
        sim.setPressureTrain(false);
        return "autonomous Standby loop requested";
      }
    },
    resetModeOverride: {
      label: "reset mode override",
      wireCommand: "MODE OVERRIDE RESET",
      aliases: ["reset mode override", "override reset", "reset override"],
      effect: function (sim) {
        sim.manual_mode_overwrite = false;
        return "mode override reset";
      }
    },
    startPressurisation: {
      label: "start pressurisation",
      wireCommand: "PRESSURE ON",
      aliases: ["start pressurisation", "start pressurization", "pressurise", "pressurize"],
      effect: function (sim) {
        sim.pressurisationActive = true;
        sim.setPressureTrain(true);
        sim.outletValveOpen = false;
        sim.setPeripheral("outletValve", false);
        return "pressure MCU enabled pumps and compressor";
      }
    },
    stopPressurisation: {
      label: "stop pressurisation",
      wireCommand: "PRESSURE OFF",
      aliases: ["stop pressurisation", "stop pressurization", "stop pressure"],
      effect: function (sim) {
        sim.pressurisationActive = false;
        sim.setPressureTrain(false);
        return "pressure MCU disabled pumps and compressor";
      }
    },
    openOutletValve: {
      label: "open outlet valve",
      wireCommand: "VALVE OPEN",
      aliases: ["open outlet valve", "open valve", "outlet open"],
      effect: function (sim) {
        sim.outletValveOpen = true;
        sim.setPeripheral("outletValve", true);
        return "normally closed outlet valve commanded open";
      }
    },
    closeOutletValve: {
      label: "close outlet valve",
      wireCommand: "VALVE CLOSE",
      aliases: ["close outlet valve", "close valve", "outlet close"],
      effect: function (sim) {
        sim.outletValveOpen = false;
        sim.setPeripheral("outletValve", false);
        return "outlet valve commanded closed";
      }
    },
    enableHeating: {
      label: "enable heating",
      wireCommand: "HEATER ALL ON",
      aliases: ["enable heating", "heating on", "heat on"],
      effect: function (sim) {
        sim.heatingEnabled = true;
        sim.heaterMask |= DEFAULT_HEATER_MASK;
        return "thermal MCU heater loops enabled";
      }
    },
    disableHeating: {
      label: "disable heating",
      wireCommand: "HEATER ALL OFF",
      aliases: ["disable heating", "heating off", "heat off"],
      effect: function (sim) {
        sim.heatingEnabled = false;
        sim.heaterMask &= ~DEFAULT_HEATER_MASK;
        return "thermal MCU heater loops disabled";
      }
    },
    enableCooling: {
      label: "enable cooling",
      wireCommand: "COOLING ON",
      aliases: ["enable cooling", "cooling on", "peltier on"],
      effect: function (sim) {
        sim.coolingEnabled = true;
        sim.heaterMask |= DEFAULT_COOLER_MASK;
        return "Peltier cooler loop enabled";
      }
    },
    disableCooling: {
      label: "disable cooling",
      wireCommand: "COOLING OFF",
      aliases: ["disable cooling", "cooling off", "peltier off"],
      effect: function (sim) {
        sim.coolingEnabled = false;
        sim.heaterMask &= ~DEFAULT_COOLER_MASK;
        return "Peltier cooler loop disabled";
      }
    },
    flushChamber: {
      label: "flush chamber",
      wireCommand: "FLUSH CHAMBER",
      aliases: ["flush chamber", "flush", "purge chamber"],
      effect: function (sim) {
        sim.flushTicks = 8;
        sim.outletValveOpen = true;
        sim.setPeripheral("outletValve", true);
        sim.pressurisationActive = true;
        sim.setPressureTrain(true);
        return "flush sequence started with fresh ambient-air exchange";
      }
    },
    restartController: {
      label: "restart main controller",
      wireCommand: "REBOOT",
      aliases: ["restart main controller", "restart controller", "reboot mcu", "reboot"],
      disruptive: true,
      effect: function (sim) {
        sim.controllerRebootTicks = 4;
        sim.mode = "STANDBY";
        sim.pressurisationActive = false;
        sim.setPressureTrain(false);
        return "main MCU reboot sequence started";
      }
    },
    emergencyStop: {
      label: "emergency stop / safe shutdown",
      wireCommand: "EMERGENCY STOP",
      aliases: ["emergency stop", "safe shutdown", "shutdown", "estop", "e-stop"],
      disruptive: true,
      effect: function (sim) {
        sim.emergencyStopped = true;
        sim.mode = "SAFE";
        sim.pressurisationActive = false;
        sim.heatingEnabled = false;
        sim.coolingEnabled = false;
        sim.outletValveOpen = true;
        sim.heaterMask = 0x00;
        sim.setPressureTrain(false);
        sim.setPeripheral("outletValve", true);
        sim.forcedFaultTicks = 14;
        return "safe shutdown latched; loads disabled and outlet opened";
      }
    },
    relay1On: {
      label: "turn PDB relay 1 on",
      wireCommand: "RELAY 1 ON",
      aliases: ["relay 1 on", "pdb relay 1 on", "relay one on"],
      stateTarget: { group: "relays", key: "relay1", value: true },
      effect: function (sim) {
        sim.setRelay("relay1", true);
        return RELAY_LINES[0].pin + " commanded on";
      }
    },
    relay1Off: {
      label: "turn PDB relay 1 off",
      wireCommand: "RELAY 1 OFF",
      aliases: ["relay 1 off", "pdb relay 1 off", "relay one off"],
      stateTarget: { group: "relays", key: "relay1", value: false },
      effect: function (sim) {
        sim.setRelay("relay1", false);
        return RELAY_LINES[0].pin + " commanded off";
      }
    },
    relay2On: {
      label: "turn PDB relay 2 on",
      wireCommand: "RELAY 2 ON",
      aliases: ["relay 2 on", "pdb relay 2 on", "relay two on"],
      stateTarget: { group: "relays", key: "relay2", value: true },
      effect: function (sim) {
        sim.setRelay("relay2", true);
        return RELAY_LINES[1].pin + " commanded on";
      }
    },
    relay2Off: {
      label: "turn PDB relay 2 off",
      wireCommand: "RELAY 2 OFF",
      aliases: ["relay 2 off", "pdb relay 2 off", "relay two off"],
      stateTarget: { group: "relays", key: "relay2", value: false },
      effect: function (sim) {
        sim.setRelay("relay2", false);
        return RELAY_LINES[1].pin + " commanded off";
      }
    },
    relay3On: {
      label: "turn PDB relay 3 on",
      wireCommand: "RELAY 3 ON",
      aliases: ["relay 3 on", "pdb relay 3 on", "relay three on"],
      stateTarget: { group: "relays", key: "relay3", value: true },
      effect: function (sim) {
        sim.setRelay("relay3", true);
        return RELAY_LINES[2].pin + " commanded on";
      }
    },
    relay3Off: {
      label: "turn PDB relay 3 off",
      wireCommand: "RELAY 3 OFF",
      aliases: ["relay 3 off", "pdb relay 3 off", "relay three off"],
      stateTarget: { group: "relays", key: "relay3", value: false },
      effect: function (sim) {
        sim.setRelay("relay3", false);
        return RELAY_LINES[2].pin + " commanded off";
      }
    },
    relay4On: {
      label: "turn PDB relay 4 on",
      wireCommand: "RELAY 4 ON",
      aliases: ["relay 4 on", "pdb relay 4 on", "relay four on"],
      stateTarget: { group: "relays", key: "relay4", value: true },
      effect: function (sim) {
        sim.setRelay("relay4", true);
        return RELAY_LINES[3].pin + " commanded on";
      }
    },
    relay4Off: {
      label: "turn PDB relay 4 off",
      wireCommand: "RELAY 4 OFF",
      aliases: ["relay 4 off", "pdb relay 4 off", "relay four off"],
      stateTarget: { group: "relays", key: "relay4", value: false },
      effect: function (sim) {
        sim.setRelay("relay4", false);
        return RELAY_LINES[3].pin + " commanded off";
      }
    },
    pump1On: {
      label: "turn vacuum pump 1 on",
      wireCommand: "PUMP 1 ON",
      aliases: ["pump 1 on", "vacuum pump 1 on", "vac pump 1 on"],
      stateTarget: { group: "peripherals", key: "pump1", value: true },
      effect: function (sim) {
        sim.setPeripheral("pump1", true);
        return "vacuum pump 1 commanded on";
      }
    },
    pump1Off: {
      label: "turn vacuum pump 1 off",
      wireCommand: "PUMP 1 OFF",
      aliases: ["pump 1 off", "vacuum pump 1 off", "vac pump 1 off"],
      stateTarget: { group: "peripherals", key: "pump1", value: false },
      effect: function (sim) {
        sim.setPeripheral("pump1", false);
        return "vacuum pump 1 commanded off";
      }
    },
    pump2On: {
      label: "turn vacuum pump 2 on",
      wireCommand: "PUMP 2 ON",
      aliases: ["pump 2 on", "vacuum pump 2 on", "vac pump 2 on"],
      stateTarget: { group: "peripherals", key: "pump2", value: true },
      effect: function (sim) {
        sim.setPeripheral("pump2", true);
        return "vacuum pump 2 commanded on";
      }
    },
    pump2Off: {
      label: "turn vacuum pump 2 off",
      wireCommand: "PUMP 2 OFF",
      aliases: ["pump 2 off", "vacuum pump 2 off", "vac pump 2 off"],
      stateTarget: { group: "peripherals", key: "pump2", value: false },
      effect: function (sim) {
        sim.setPeripheral("pump2", false);
        return "vacuum pump 2 commanded off";
      }
    },
    compressorOn: {
      label: "turn compressor on",
      wireCommand: "COMPRESSOR ON",
      aliases: ["compressor on"],
      stateTarget: { group: "peripherals", key: "compressor", value: true },
      effect: function (sim) {
        sim.setPeripheral("compressor", true);
        return "compressor commanded on";
      }
    },
    compressorOff: {
      label: "turn compressor off",
      wireCommand: "COMPRESSOR OFF",
      aliases: ["compressor off"],
      stateTarget: { group: "peripherals", key: "compressor", value: false },
      effect: function (sim) {
        sim.setPeripheral("compressor", false);
        return "compressor commanded off";
      }
    },
    outletValveOpen: {
      label: "open outlet valve override",
      wireCommand: "VALVE OPEN",
      aliases: ["valve open", "outlet valve open", "manual valve open"],
      stateTarget: { group: "peripherals", key: "outletValve", value: true },
      effect: function (sim) {
        sim.outletValveOpen = true;
        sim.setPeripheral("outletValve", true);
        return "outlet valve manual override open";
      }
    },
    outletValveClose: {
      label: "close outlet valve override",
      wireCommand: "VALVE CLOSE",
      aliases: ["valve close", "valve closed", "outlet valve close", "outlet valve closed", "manual valve close"],
      stateTarget: { group: "peripherals", key: "outletValve", value: false },
      effect: function (sim) {
        sim.outletValveOpen = false;
        sim.setPeripheral("outletValve", false);
        return "outlet valve manual override closed";
      }
    },
    k96On: {
      label: "turn K96 on",
      wireCommand: "K96 ON",
      aliases: ["k96 on", "turn k96 on"],
      stateTarget: { group: "peripherals", key: "k96", value: true },
      effect: function (sim) {
        sim.setPeripheral("k96", true);
        return "K96 commanded on";
      }
    },
    k96Off: {
      label: "turn K96 off",
      wireCommand: "K96 OFF",
      aliases: ["k96 off", "turn k96 off"],
      stateTarget: { group: "peripherals", key: "k96", value: false },
      effect: function (sim) {
        sim.setPeripheral("k96", false);
        return "K96 commanded off";
      }
    }
  };

  THERMAL_CHANNELS.forEach(function (channel) {
    ["on", "off"].forEach(function (action) {
      const commandId = "heater" + (channel.bit + 1) + action.charAt(0).toUpperCase() + action.slice(1);
      const enabled = action === "on";
      COMMANDS[commandId] = {
        label: (enabled ? "turn " : "turn ") + channel.label + " " + action,
        wireCommand: "HEATER " + action.toUpperCase() + " " + (channel.bit + 1),
        aliases: [],
        heaterBit: channel.bit,
        heaterEnabled: enabled,
        effect: function (sim) {
          if (enabled) {
            sim.heaterMask |= 1 << channel.bit;
          } else {
            sim.heaterMask &= ~(1 << channel.bit);
          }
          return channel.label + " commanded " + action;
        }
      };
    });
  });

  const dom = {
    overallHealth: document.getElementById("overallHealth"),
    healthDetails: document.getElementById("healthDetails"),
    missionMode: document.getElementById("missionMode"),
    linkState: document.getElementById("linkState"),
    lastFrameAge: document.getElementById("lastFrameAge"),
    storageState: document.getElementById("storageState"),
    controllerState: document.getElementById("controllerState"),
    logFeed: document.getElementById("logFeed"),
    frameNumber: document.getElementById("frameNumber"),
    latencyValue: document.getElementById("latencyValue"),
    methaneValue: document.getElementById("methaneValue"),
    chamberPressureValue: document.getElementById("chamberPressureValue"),
    chamberTempValue: document.getElementById("chamberTempValue"),
    humidityValue: document.getElementById("humidityValue"),
    linkQualityValue: document.getElementById("linkQualityValue"),
    metricMethane: document.getElementById("metricMethane"),
    metricPressure: document.getElementById("metricPressure"),
    metricTemperature: document.getElementById("metricTemperature"),
    metricHumidity: document.getElementById("metricHumidity"),
    metricLink: document.getElementById("metricLink"),
    commandStatus: document.getElementById("commandStatus"),
    terminalOutput: document.getElementById("terminalOutput"),
    terminalForm: document.getElementById("terminalForm"),
    terminalInput: document.getElementById("terminalInput"),
    terminalRoute: document.getElementById("terminalRoute"),
    telemetryView: document.getElementById("telemetryView"),
    systemOverviewView: document.getElementById("systemOverviewView"),
    telemetryViewButton: document.getElementById("telemetryViewButton"),
    overviewViewButton: document.getElementById("overviewViewButton"),
    gasChart: document.getElementById("gasChart"),
    pressureChart: document.getElementById("pressureChart"),
    thermalChart: document.getElementById("thermalChart"),
    ambientChart: document.getElementById("ambientChart"),
    linkChart: document.getElementById("linkChart"),
    heaterActuationChart: document.getElementById("heaterActuationChart"),
    diagramAmbientValue: document.getElementById("diagramAmbientValue"),
    diagramPreheaterValue: document.getElementById("diagramPreheaterValue"),
    diagramPump1Value: document.getElementById("diagramPump1Value"),
    diagramPump2Value: document.getElementById("diagramPump2Value"),
    diagramCompressorValue: document.getElementById("diagramCompressorValue"),
    diagramChamberValue: document.getElementById("diagramChamberValue"),
    diagramValveValue: document.getElementById("diagramValveValue"),
    diagramLinkValue: document.getElementById("diagramLinkValue"),
    diagramMainValue: document.getElementById("diagramMainValue"),
    diagramPressureMcuValue: document.getElementById("diagramPressureMcuValue"),
    diagramThermalMcuValue: document.getElementById("diagramThermalMcuValue"),
    diagramStorageValue: document.getElementById("diagramStorageValue"),
    diagramHeaterValue: document.getElementById("diagramHeaterValue"),
    diagramCoolerValue: document.getElementById("diagramCoolerValue"),
    diagramRelay1Value: document.getElementById("diagramRelay1Value"),
    diagramRelay2Value: document.getElementById("diagramRelay2Value"),
    diagramRelay3Value: document.getElementById("diagramRelay3Value"),
    diagramRelay4Value: document.getElementById("diagramRelay4Value")
  };

  const commandAliases = buildCommandAliases(COMMANDS);
  const history = [];
  let latestTelemetry = null;
  let latestMissionMode = null;
  let lastGoodFrameAt = 0;
  let previousHealth = "unknown";
  let previousLinkStatus = "unknown";
  let usingGateway = false;
  let legacyProtocol = false;
  const commandedState = {
    relays: {
      relay1: false,
      relay2: false,
      relay3: false,
      relay4: false
    },
    peripherals: {
      pump1: false,
      pump2: false,
      compressor: false,
      outletValve: false,
      k96: false
    },
    heaterMask: DEFAULT_HEATER_MASK,
    coolerMask: 0x00
  };

  class OperationsLog {
    constructor(feedElement) {
      this.feedElement = feedElement;
      this.entries = [];
    }

    add(level, title, message) {
      const entry = {
        level: level || "info",
        title: title,
        message: message,
        time: new Date()
      };

      this.entries.unshift(entry);
      this.entries = this.entries.slice(0, 90);
      this.render();
    }

    render() {
      this.feedElement.textContent = "";
      const fragment = document.createDocumentFragment();

      this.entries.forEach(function (entry) {
        const item = document.createElement("li");
        item.className = "log-entry " + entry.level;

        const time = document.createElement("time");
        time.dateTime = entry.time.toISOString();
        time.textContent = formatClock(entry.time);

        const body = document.createElement("div");
        const title = document.createElement("strong");
        const message = document.createElement("p");
        title.textContent = entry.title;
        message.textContent = entry.message;

        body.append(title, message);
        item.append(time, body);
        fragment.append(item);
      });

      this.feedElement.append(fragment);
    }
  }

  class Terminal {
    constructor(outputElement) {
      this.outputElement = outputElement;
    }

    write(text, level) {
      const line = document.createElement("p");
      const time = document.createElement("span");
      const body = document.createElement("span");
      const className = level === "error" ? "error-line" : level === "warn" ? "warn-line" : "";

      line.className = "terminal-line " + className;
      time.className = "time";
      time.textContent = "[" + formatClock(new Date()) + "] ";
      body.textContent = text;
      line.append(time, body);
      this.outputElement.append(line);
      this.outputElement.scrollTop = this.outputElement.scrollHeight;
    }

    command(text) {
      const line = document.createElement("p");
      line.className = "terminal-line command-line";
      line.textContent = "> " + text;
      this.outputElement.append(line);
      this.outputElement.scrollTop = this.outputElement.scrollHeight;
    }

    clear() {
      this.outputElement.textContent = "";
    }
  }
  class MockMirageSimulator {
    constructor() {
      this.tick = 0;
      this.seq = 0;
      this.mode = "STANDBY";
      this.health = "healthy";
      this.linkStatus = "ONLINE";
      this.pressurisationActive = false;
      this.outletValveOpen = false;
      this.heatingEnabled = true;
      this.coolingEnabled = false;
      this.heaterMask = DEFAULT_HEATER_MASK;
      this.relays = {
        relay1: false,
        relay2: false,
        relay3: false,
        relay4: false
      };
      this.peripherals = {
        pump1: false,
        pump2: false,
        compressor: false,
        outletValve: false,
        k96: false
      };
      this.emergencyStopped = false;
      this.controllerRebootTicks = 0;
      this.flushTicks = 0;
      this.forcedFaultTicks = 0;
      this.scenario = { name: "healthy", remaining: 18 };

      this.values = {
        methaneRaw: 28400,
        co2Raw: 32700,
        waterRaw: 21200,
        chamberPressureBar: 2.82,
        chamberTempC: 21.5,
        electronicsTempC: 26.0,
        humidityRh: 38,
        ambientPressureBar: 1.012,
        ambientTempC: 6,
        linkQuality: 98,
        latencyMs: 74,
        storageFreePct: 93
      };
    }

    setRelay(relayId, enabled) {
      if (Object.prototype.hasOwnProperty.call(this.relays, relayId)) {
        this.relays[relayId] = Boolean(enabled);
      }
    }

    setPeripheral(peripheralId, enabled) {
      if (Object.prototype.hasOwnProperty.call(this.peripherals, peripheralId)) {
        this.peripherals[peripheralId] = Boolean(enabled);
      }

      if (peripheralId === "outletValve") {
        this.outletValveOpen = Boolean(enabled);
      }

      this.pressurisationActive = this.peripherals.pump1 || this.peripherals.pump2 || this.peripherals.compressor;
    }

    setPressureTrain(enabled) {
      this.peripherals.pump1 = Boolean(enabled);
      this.peripherals.pump2 = Boolean(enabled);
      this.peripherals.compressor = Boolean(enabled);
      this.pressurisationActive = Boolean(enabled);
    }

    generateFrame() {
      this.tick += 1;

      if (this.controllerRebootTicks > 0) {
        this.controllerRebootTicks -= 1;
        return this.dropoutFrame("main MCU rebooting");
      }

      this.advanceScenario();

      if (this.scenario.name === "dropout") {
        this.scenario.remaining -= 1;
        return this.dropoutFrame("E-Link frame missing");
      }

      this.seq += 1;
      this.updateEnvironment();

      const health = this.evaluateHealth();
      this.health = health;
      this.linkStatus = this.values.linkQuality < 62 ? "DEGRADED" : "ONLINE";

      return {
        valid: true,
        timestamp: Date.now(),
        seq: this.seq,
        mode: this.mode,
        health: health,
        linkStatus: this.linkStatus,
        linkQuality: this.values.linkQuality,
        latencyMs: this.values.latencyMs,
        methaneRaw: this.values.methaneRaw,
        co2Raw: this.values.co2Raw,
        waterRaw: this.values.waterRaw,
        chamberPressureBar: this.values.chamberPressureBar,
        chamberTempC: this.values.chamberTempC,
        electronicsTempC: this.values.electronicsTempC,
        humidityRh: this.values.humidityRh,
        ambientPressureBar: this.values.ambientPressureBar,
        ambientTempC: this.values.ambientTempC,
        pump1DutyPct: this.peripherals.pump1 ? clamp(68 + noise(8), 0, 100) : 0,
        pump2DutyPct: this.peripherals.pump2 ? clamp(66 + noise(8), 0, 100) : 0,
        compressorDutyPct: this.peripherals.compressor ? clamp(58 + noise(10), 0, 100) : 0,
        heaterDutyPct: this.heatingEnabled ? clamp(36 + (22 - this.values.chamberTempC) * 4 + noise(5), 0, 100) : 0,
        heater1ActuationPct: this.heaterMask & (1 << 0) ? 100 : 0,
        heater2ActuationPct: this.heaterMask & (1 << 1) ? 100 : 0,
        heater3ActuationPct: this.heaterMask & (1 << 2) ? 100 : 0,
        heater4ActuationPct: this.heaterMask & (1 << 3) ? 100 : 0,
        heater5ActuationPct: this.heaterMask & (1 << 4) ? 100 : 0,
        heater6ActuationPct: this.heaterMask & (1 << 5) ? 100 : 0,
        heater7ActuationPct: this.heaterMask & (1 << 6) ? 100 : 0,
        heater8ActuationPct: this.heaterMask & (1 << 7) ? 100 : 0,
        coolerDutyPct: this.coolingEnabled ? clamp(26 + (this.values.chamberTempC - 24) * 4 + noise(5), 0, 100) : 0,
        outletValveOpen: this.outletValveOpen,
        pressureSystemOn: this.pressurisationActive,
        heaterMask: this.heaterMask,
        relayLines: Object.assign({}, this.relays),
        peripherals: Object.assign({}, this.peripherals),
        onboardLogging: true,
        storageFreePct: this.values.storageFreePct,
        controller: "MAIN_MCU_READY",
        controllerReady: true,
        pressureState: this.flushTicks > 0 ? 4 : this.pressurisationActive ? 1 : 0,
        activeTask: this.flushTicks > 0 ? "FLUSH_CHAMBER" : this.pressurisationActive ? "PRESSURISATION" : null,
        scenario: this.scenario.name,
        statusText: this.statusText()
      };
    }

    dropoutFrame(reason) {
      this.linkStatus = "DROPOUT";
      this.values.linkQuality = clamp(this.values.linkQuality - 24 + noise(3), 0, 100);
      this.values.latencyMs = 0;

      return {
        valid: false,
        timestamp: Date.now(),
        seq: this.seq,
        mode: this.mode,
        health: this.emergencyStopped ? "fault" : "dropout",
        linkStatus: "DROPOUT",
        linkQuality: 0,
        latencyMs: 0,
        dropoutReason: reason,
        controller: this.controllerRebootTicks > 0 ? "MAIN_MCU_RESTARTING" : "LINK_DROP",
        controllerReady: false
      };
    }

    advanceScenario() {
      if (this.forcedFaultTicks > 0) {
        this.forcedFaultTicks -= 1;
        this.scenario = { name: "safe-shutdown", remaining: this.forcedFaultTicks };
        return;
      }

      this.scenario.remaining -= 1;
      if (this.scenario.remaining > 0) {
        return;
      }

      const roll = Math.random();
      if (roll < 0.56) {
        this.scenario = { name: "healthy", remaining: randInt(12, 26) };
      } else if (roll < 0.70) {
        this.scenario = { name: "humidity-warning", remaining: randInt(7, 14) };
      } else if (roll < 0.82) {
        this.scenario = { name: "pressure-warning", remaining: randInt(7, 15) };
      } else if (roll < 0.91) {
        this.scenario = { name: "thermal-warning", remaining: randInt(6, 13) };
      } else if (roll < 0.97) {
        this.scenario = { name: "dropout", remaining: randInt(3, 7) };
      } else {
        this.scenario = { name: "fault", remaining: randInt(5, 10) };
      }
    }

    updateEnvironment() {
      const descentCycle = Math.sin(this.tick / 520);
      const ambientTarget = clamp(990 - this.tick * 0.55 + descentCycle * 18, 72, 1015);
      this.values.ambientPressureBar = approach(this.values.ambientPressureBar, ambientTarget, 0.012) + noise(0.5);
      const altitudeFactor = clamp((1.010 - this.values.ambientPressureBar) / 0.935, 0, 1);
      this.values.ambientTempC = clamp(8 - altitudeFactor * 88 + Math.sin(this.tick / 40) * 2 + noise(0.6), -82, 16);

      let pressureTarget = 1.08;
      if (this.mode === "MEASUREMENTS" || this.pressurisationActive) {
        pressureTarget = 3.0;
      } else if (!this.outletValveOpen) {
        pressureTarget = 2.1;
      }

      if (this.outletValveOpen && !this.pressurisationActive) {
        pressureTarget = 1.08;
      }

      if (this.scenario.name === "pressure-warning") {
        pressureTarget += Math.random() > 0.5 ? 0.32 : -0.36;
      }

      if (this.scenario.name === "fault") {
        pressureTarget += 0.72;
      }

      if (this.scenario.name === "safe-shutdown") {
        pressureTarget = 1.05;
      }

      if (this.flushTicks > 0) {
        this.flushTicks -= 1;
        pressureTarget = 2.68 + noise(0.08);
        this.values.humidityRh = approach(this.values.humidityRh, 28, 0.22);
        this.values.waterRaw = approach(this.values.waterRaw, 18800, 0.25);
        if (this.flushTicks === 0) {
          this.outletValveOpen = false;
          this.setPeripheral("outletValve", false);
        }
      }

      this.values.chamberPressureBar = clamp(approach(this.values.chamberPressureBar, pressureTarget, 0.18) + noise(0.015), 0.85, 4.25);

      let chamberTarget = 21.5;
      if (!this.heatingEnabled) {
        chamberTarget -= 8 + altitudeFactor * 12;
      }
      if (this.coolingEnabled) {
        chamberTarget -= 4;
      }
      if (this.pressurisationActive) {
        chamberTarget += 1.8;
      }
      if (this.scenario.name === "thermal-warning") {
        chamberTarget += Math.random() > 0.45 ? 21 : -15;
      }
      if (this.scenario.name === "fault") {
        chamberTarget += 28;
      }
      if (this.scenario.name === "safe-shutdown") {
        chamberTarget = 13;
      }

      this.values.chamberTempC = clamp(approach(this.values.chamberTempC, chamberTarget, 0.08) + noise(0.12), -18, 65);
      this.values.electronicsTempC = clamp(approach(this.values.electronicsTempC, this.values.chamberTempC + 4 + (this.pressurisationActive ? 3 : 0), 0.06) + noise(0.16), -12, 72);

      let humidityTarget = 38 + altitudeFactor * 10;
      if (this.scenario.name === "humidity-warning") {
        humidityTarget = 74 + noise(5);
      }
      if (this.values.chamberTempC < 14) {
        humidityTarget += 11;
      }
      if (this.outletValveOpen && this.flushTicks > 0) {
        humidityTarget = 28;
      }

      this.values.humidityRh = clamp(approach(this.values.humidityRh, humidityTarget, 0.11) + noise(0.8), 12, 94);
      this.values.waterRaw = clamp(approach(this.values.waterRaw, 17800 + this.values.humidityRh * 92 + altitudeFactor * 1100, 0.14) + noise(70), 0, 65535);

      const gasPulse = Math.sin(this.tick / 18) * 180;
      this.values.methaneRaw = clamp(28400 + altitudeFactor * 900 + gasPulse + noise(80), 0, 65535);
      this.values.co2Raw = clamp(32700 - altitudeFactor * 700 + Math.sin(this.tick / 22) * 210 + noise(90), 0, 65535);

      let linkTarget = 97;
      if (this.scenario.name === "humidity-warning" || this.scenario.name === "pressure-warning") {
        linkTarget = 88;
      }
      if (this.scenario.name === "thermal-warning") {
        linkTarget = 82;
      }
      if (this.scenario.name === "fault") {
        linkTarget = 67;
      }
      if (this.scenario.name === "safe-shutdown") {
        linkTarget = 92;
      }

      this.values.linkQuality = clamp(approach(this.values.linkQuality, linkTarget, 0.21) + noise(2.8), 18, 100);
      this.values.latencyMs = Math.round(clamp(70 + (100 - this.values.linkQuality) * 3 + noise(15), 42, 420));
      this.values.storageFreePct = clamp(this.values.storageFreePct - 0.006, 12, 100);
    }

    evaluateHealth() {
      if (this.emergencyStopped || this.scenario.name === "safe-shutdown") {
        return "fault";
      }

      if (
        this.values.chamberPressureBar > 3.55 ||
        this.values.chamberPressureBar < 2.35 ||
        this.values.chamberTempC > 50 ||
        this.values.chamberTempC < 5 ||
        this.values.electronicsTempC > 58 ||
        this.values.humidityRh > 84
      ) {
        return "fault";
      }

      if (
        this.values.chamberPressureBar > 3.22 ||
        this.values.chamberPressureBar < 2.74 ||
        this.values.chamberTempC > 40 ||
        this.values.chamberTempC < 15 ||
        this.values.electronicsTempC > 48 ||
        this.values.humidityRh > 65 ||
        this.values.linkQuality < 75
      ) {
        return "warning";
      }

      return "healthy";
    }

    statusText() {
      if (this.emergencyStopped) {
        return "safe shutdown latched";
      }
      if (this.flushTicks > 0) {
        return "chamber flush active";
      }
      if (this.scenario.name === "humidity-warning") {
        return "humidity protection loop near threshold";
      }
      if (this.scenario.name === "pressure-warning") {
        return "pressure controller outside target band";
      }
      if (this.scenario.name === "thermal-warning") {
        return "thermal control margin reduced";
      }
      if (this.scenario.name === "fault") {
        return "fault injection active in mock telemetry";
      }
      return "nominal autonomous supervision";
    }

    applyCommand(commandId) {
      const definition = COMMANDS[commandId];
      if (!definition) {
        return "unknown command";
      }
      return definition.effect(this);
    }
  }

  class MockTelemetrySource {
    constructor(simulator, onFrame, onLogEvent) {
      this.simulator = simulator;
      this.onFrame = onFrame;
      this.onLogEvent = onLogEvent;
      this.timer = null;
    }

    start() {
      if (this.timer) {
        return;
      }
      this.emit();
      this.timer = window.setInterval(this.emit.bind(this), TELEMETRY_PERIOD_MS);
    }

    stop() {
      if (this.timer) {
        window.clearInterval(this.timer);
        this.timer = null;
      }
    }

    emit() {
      const sample = this.simulator.generateFrame();
      this.onFrame(sample);
      this.logFrame(sample);
    }

    logFrame(sample) {
      if (!sample.valid) {
        if (previousLinkStatus !== "DROPOUT") {
          this.onLogEvent("dropout", "Telemetry dropout", sample.dropoutReason + "; operator view is holding last valid data");
        }
        previousLinkStatus = "DROPOUT";
        return;
      }

      this.onLogEvent("info", "Telemetry frame " + sample.seq + " received", sample.statusText);

      if (previousLinkStatus === "DROPOUT") {
        this.onLogEvent("info", "E-Link recovered", "live telemetry resumed at frame " + sample.seq);
      }

      if (sample.health !== previousHealth && previousHealth !== "unknown") {
        const level = sample.health === "healthy" ? "info" : sample.health === "warning" ? "warn" : "fault";
        this.onLogEvent(level, "Health state changed", "system is now " + sample.health.toUpperCase());
      }

      previousHealth = sample.health;
      previousLinkStatus = sample.linkStatus;
    }
  }

  class MockCommandRouter {
    constructor(simulator) {
      this.simulator = simulator;
      this.listeners = [];
    }

    on(listener) {
      this.listeners.push(listener);
    }

    emit(event) {
      this.listeners.forEach(function (listener) {
        listener(event);
      });
    }

    send(commandId, origin) {
      const definition = COMMANDS[commandId];
      const requestId = "CMD-" + Math.floor(1000 + Math.random() * 9000);
      const delay = randInt(420, 1900);
      const linkPenalty = this.simulator.linkStatus === "DROPOUT" ? 0.34 : 0;
      const failChance = definition && definition.disruptive ? 0.1 + linkPenalty : 0.055 + linkPenalty;

      if (!definition) {
        return Promise.reject(new Error("unknown command"));
      }

      this.emit({
        type: "queued",
        commandId: commandId,
        label: definition.label,
        requestId: requestId,
        origin: origin,
        delay: delay
      });

      return new Promise((resolve, reject) => {
        window.setTimeout(() => {
          if (Math.random() < failChance) {
            const error = {
              type: "nack",
              commandId: commandId,
              label: definition.label,
              requestId: requestId,
              origin: origin,
              message: "mock uplink did not receive a valid ACK before timeout"
            };
            this.emit(error);
            reject(error);
            return;
          }

          const result = this.simulator.applyCommand(commandId);
          const ack = {
            type: "ack",
            commandId: commandId,
            label: definition.label,
            requestId: requestId,
            origin: origin,
            message: result
          };
          this.emit(ack);
          resolve(ack);
        }, delay);
      });
    }
  }

  class GatewayCommandRouter {
    constructor() {
      this.listeners = [];
    }

    on(listener) {
      this.listeners.push(listener);
    }

    emit(event) {
      this.listeners.forEach(function (listener) {
        listener(event);
      });
    }

    send(commandId, origin) {
      const definition = COMMANDS[commandId];
      const requestId = "CMD-" + Math.floor(1000 + Math.random() * 9000);

      if (!definition) {
        return Promise.reject(new Error("unknown command"));
      }

      if (!canUseGateway()) {
        return Promise.reject(new Error("ground-station gateway is unavailable from this page"));
      }

      this.emit({
        type: "queued",
        commandId: commandId,
        label: definition.label,
        wireCommand: definition.wireCommand,
        requestId: requestId,
        origin: origin,
        delay: 0
      });

      return window.fetch("/api/command", {
        method: "POST",
        headers: {
          "Content-Type": "application/json"
        },
        body: JSON.stringify({
          requestId: requestId,
          commandId: commandId,
          label: definition.label,
          origin: origin,
          wireCommand: definition.wireCommand
        })
      })
        .then(function (response) {
          return response.json().then(function (body) {
            if (!response.ok || !body.ok) {
              const error = {
                type: "nack",
                commandId: commandId,
                label: definition.label,
                wireCommand: definition.wireCommand,
                requestId: requestId,
                origin: origin,
                message: body.message || "gateway rejected command"
              };
              throw error;
            }

            return {
              type: "ack",
              commandId: commandId,
              label: definition.label,
              wireCommand: definition.wireCommand,
              requestId: requestId,
              origin: origin,
              message: body.message || "sent to payload TCP command socket"
            };
          });
        })
        .then((ack) => {
          this.emit(ack);
          return ack;
        })
        .catch((error) => {
          const event = error && error.type === "nack" ? error : {
            type: "nack",
            commandId: commandId,
            label: definition.label,
            wireCommand: definition.wireCommand,
            requestId: requestId,
            origin: origin,
            message: error && error.message ? error.message : "gateway command send failed"
          };
          this.emit(event);
          return Promise.reject(event);
        });
    }
  }

  class GatewayTelemetrySource {
    constructor(onFrame, onLogEvent, onConnect, onDisconnect) {
      this.onFrame = onFrame;
      this.onLogEvent = onLogEvent;
      this.onConnect = onConnect;
      this.onDisconnect = onDisconnect;
      this.source = null;
      this.connected = false;
    }

    start() {
      if (!canUseGateway() || !window.EventSource) {
        return false;
      }

      this.source = new window.EventSource("/api/telemetry");
      this.source.addEventListener("telemetry", this.handleTelemetry.bind(this));
      this.source.addEventListener("gateway", this.handleGatewayEvent.bind(this));
      this.source.onerror = this.handleError.bind(this);
      return true;
    }

    stop() {
      if (this.source) {
        this.source.close();
        this.source = null;
      }
      this.connected = false;
    }

    handleTelemetry(event) {
      let sample;
      try {
        sample = JSON.parse(event.data);
      } catch (error) {
        this.onLogEvent("warn", "Gateway telemetry rejected", "received malformed JSON from local gateway");
        return;
      }

      if (!this.connected) {
        this.connected = true;
        this.onConnect();
      }

      this.onFrame(sample);
      this.logFrame(sample);
    }

    handleGatewayEvent(event) {
      let status;
      try {
        status = JSON.parse(event.data);
      } catch (error) {
        return;
      }

      const nextLegacyProtocol = Number.isFinite(status.packetSize) && status.packetSize < EXPECTED_PACKET_SIZE;
      if (nextLegacyProtocol && !legacyProtocol) {
        this.onLogEvent("warn", "Payload protocol update required", "live gateway still uses the legacy " + status.packetSize + "-byte status packet");
        terminal.write("payload firmware/gateway update required for SD, controller, raw detector, and task telemetry", "warn");
      }
      legacyProtocol = nextLegacyProtocol;
      if (latestTelemetry) {
        renderTelemetry(latestTelemetry);
      }

      if (status.payloadConnected) {
        if (activeCommandRouter !== gatewayCommandRouter) {
          activeCommandRouter = gatewayCommandRouter;
          terminal.write("payload TCP command socket connected through local gateway.");
          this.onLogEvent("info", "Payload command socket connected", "GUI commands now route to the local gateway");
        }

        setChip(dom.linkState, "E-Link online", "healthy");

        if (!this.connected) {
          this.onLogEvent("info", "E-Link gateway ready", "waiting for first decoded payload status frame");
        }
        return;
      }

      if (activeCommandRouter === gatewayCommandRouter) {
        if (MOCK_DATA_ENABLED) {
          activeCommandRouter = mockCommandRouter;
          terminal.write("payload command socket disconnected; local mock route resumed", "warn");
          this.onLogEvent("dropout", "Payload command socket disconnected", "GUI commands are back on the local mock route");
        } else {
          terminal.write("payload command socket disconnected; telemetry halted until reconnect", "warn");
          this.onLogEvent("dropout", "Payload command socket disconnected", "mock telemetry fallback is disabled");
        }
      }
      setChip(dom.linkState, "E-Link dropout", "dropout");
      setChip(dom.missionMode, resolveMissionMode(latestTelemetry), "neutral");
      dom.controllerState.textContent = "No telemetry";
      renderActiveTask(null);

      if (this.connected) {
        this.connected = false;
        this.onDisconnect();
      }
    }

    handleError() {
      if (this.connected) {
        this.onLogEvent("dropout", "Ground-station gateway disconnected", MOCK_DATA_ENABLED ? "browser SSE stream dropped; local mock telemetry resumed" : "browser SSE stream dropped; telemetry is unavailable");
        this.connected = false;
        this.onDisconnect();
      }

      setChip(dom.linkState, "E-Link dropout", "dropout");
      setChip(dom.missionMode, resolveMissionMode(latestTelemetry), "neutral");
      dom.controllerState.textContent = "No telemetry";
      renderActiveTask(null);
    }

    logFrame(sample) {
      if (!sample.valid) {
        if (previousLinkStatus !== "DROPOUT") {
          this.onLogEvent("dropout", "Telemetry dropout", sample.dropoutReason + "; operator view is holding last valid data");
        }
        previousLinkStatus = "DROPOUT";
        return;
      }

      this.onLogEvent("info", "Telemetry frame " + sample.seq + " received", sample.statusText || "payload status packet decoded");

      if (previousLinkStatus === "DROPOUT") {
        this.onLogEvent("info", "E-Link recovered", "live telemetry resumed at frame " + sample.seq);
      }

      if (sample.health !== previousHealth && previousHealth !== "unknown") {
        const level = sample.health === "healthy" ? "info" : sample.health === "warning" ? "warn" : "fault";
        this.onLogEvent(level, "Health state changed", "system is now " + sample.health.toUpperCase());
      }

      previousHealth = sample.health;
      previousLinkStatus = sample.linkStatus;
    }
  }

  const log = new OperationsLog(dom.logFeed);
  const terminal = new Terminal(dom.terminalOutput);
  const simulator = new MockMirageSimulator();
  const mockCommandRouter = new MockCommandRouter(simulator);
  const gatewayCommandRouter = new GatewayCommandRouter();
  const mockTelemetrySource = new MockTelemetrySource(simulator, handleFrame, function (level, title, message) {
    log.add(level, title, message);
  });
  const gatewayTelemetrySource = new GatewayTelemetrySource(
    handleFrame,
    function (level, title, message) {
      log.add(level, title, message);
    },
    handleGatewayConnected,
    handleGatewayDisconnected
  );
  let activeCommandRouter = mockCommandRouter;

  function init() {
    terminal.write(MOCK_DATA_ENABLED ? "MIRAGE ground-station terminal ready on local mock route." : "MIRAGE ground-station terminal ready; waiting for gateway telemetry.");
    terminal.write("Type help for available manual commands.");
    log.add("info", "Ground station initialized", MOCK_DATA_ENABLED ? "mock telemetry and mock uplink adapters active" : "mock telemetry disabled; awaiting payload connection");

    bindCommands();
    bindHealthDetails();
    bindTerminal();
    bindViewSwitch();
    mockCommandRouter.on(handleCommandEvent);
    gatewayCommandRouter.on(handleCommandEvent);
    if (MOCK_DATA_ENABLED) {
      mockTelemetrySource.start();
    }
    gatewayTelemetrySource.start();
    window.setInterval(updateFrameAge, 1000);
    window.addEventListener("resize", drawAllCharts);
  }

  function bindCommands() {
    document.querySelectorAll("[data-command]").forEach(function (button) {
      button.addEventListener("click", function () {
        const commandId = button.dataset.command;
        if (button.dataset.confirm && !window.confirm(button.dataset.confirm)) {
          return;
        }

        button.disabled = true;
        sendCommand(commandId, "button")
          .catch(function () {
            return undefined;
          })
          .finally(function () {
            button.disabled = false;
          });
      });
    });

    document.querySelectorAll("[data-toggle]").forEach(function (button) {
      button.addEventListener("click", function () {
        const pressed = button.getAttribute("aria-pressed") === "true";
        const commandId = pressed ? button.dataset.commandOff : button.dataset.commandOn;
        const confirmMessage = pressed ? button.dataset.confirmOff : button.dataset.confirmOn;

        if (confirmMessage && !window.confirm(confirmMessage)) {
          return;
        }

        button.disabled = true;
        sendCommand(commandId, "button")
          .catch(function () {
            return undefined;
          })
          .finally(function () {
            button.disabled = false;
          });
      });
    });

    document.querySelectorAll("[data-heater-select]").forEach(function (button) {
      button.addEventListener("click", function () {
        const selectAll = button.dataset.heaterSelect === "all";
        document.querySelectorAll("[data-heater-checkbox]").forEach(function (checkbox) {
          checkbox.checked = selectAll;
        });
      });
    });

    document.querySelectorAll("[data-heater-shortcut]").forEach(function (button) {
      button.addEventListener("click", function () {
        const shortcut = button.dataset.heaterShortcut;
        const commandId = shortcut === "all-heaters-on"
          ? "enableHeating"
          : shortcut === "all-heaters-off"
            ? "disableHeating"
            : shortcut === "all-coolers-on"
              ? "enableCooling"
              : "disableCooling";

        button.disabled = true;
        sendCommand(commandId, "button")
          .catch(function () {
            return undefined;
          })
          .finally(function () {
            button.disabled = false;
          });
      });
    });

    document.querySelectorAll("[data-heater-action]").forEach(function (button) {
      button.addEventListener("click", function () {
        const selectedHeaters = Array.from(document.querySelectorAll("[data-heater-checkbox]:checked"))
          .map(function (checkbox) {
            return Number(checkbox.dataset.heaterCheckbox);
          });

        if (!selectedHeaters.length) {
          log.add("warn", "Heater command rejected", "Select at least one heater before sending an ON/OFF action");
          return;
        }

        const action = button.dataset.heaterAction;
        const commandIds = selectedHeaters.map(function (heaterNumber) {
          return "heater" + heaterNumber + (action === "on" ? "On" : "Off");
        });
        document.querySelectorAll("[data-heater-action]").forEach(function (actionButton) {
          actionButton.disabled = true;
        });

        commandIds.reduce(function (chain, commandId) {
          return chain.then(function () {
            return sendCommand(commandId, "button");
          });
        }, Promise.resolve())
          .catch(function () {
            return undefined;
          })
          .finally(function () {
            document.querySelectorAll("[data-heater-action]").forEach(function (actionButton) {
              actionButton.disabled = false;
            });
          });
      });
    });
  }

  function bindHealthDetails() {
    dom.overallHealth.addEventListener("click", function () {
      const expanded = dom.overallHealth.getAttribute("aria-expanded") === "true";
      dom.overallHealth.setAttribute("aria-expanded", String(!expanded));
      dom.healthDetails.hidden = expanded;
    });

    document.addEventListener("click", function (event) {
      if (!event.target.closest(".health-control")) {
        dom.overallHealth.setAttribute("aria-expanded", "false");
        dom.healthDetails.hidden = true;
      }
    });
  }

  function bindTerminal() {
    dom.terminalForm.addEventListener("submit", function (event) {
      event.preventDefault();
      const raw = dom.terminalInput.value.trim();
      if (!raw) {
        return;
      }

      dom.terminalInput.value = "";
      terminal.command(raw);
      handleTerminalCommand(raw);
    });

    const heaterConfigForm = document.getElementById("heaterConfigForm");
    if (heaterConfigForm) {
      heaterConfigForm.addEventListener("submit", function (event) {
        event.preventDefault();

        const selectedHeaters = Array.from(document.querySelectorAll("[data-heater-checkbox]:checked"))
          .map(function (checkbox) {
            return Number(checkbox.dataset.heaterCheckbox);
          });

        if (!selectedHeaters.length) {
          log.add("warn", "Heater config rejected", "Select at least one heater before applying a mode or setpoint");
          return;
        }

        const mode = document.getElementById("heaterConfigMode").value;
        const targetInput = document.getElementById("heaterConfigTarget");
        const dutyInput = document.getElementById("heaterConfigDuty");
        const targetValue = parseFloat(targetInput.value);
        const dutyValue = parseFloat(dutyInput.value);

        const commandIds = selectedHeaters.map(function (heaterNumber) {
          const normalizedValue = (mode === "MANUAL")
            ? "heater " + heaterNumber + " mode manual duty " + String(Math.max(0, Math.min(100, Number.isFinite(dutyValue) ? dutyValue : 0)))
            : "heater " + heaterNumber + " mode " + mode.toLowerCase() + " target " + String(Number.isFinite(targetValue) ? targetValue : 25);

          return parseHeaterControlCommand(normalizeCommand(normalizedValue));
        }).filter(Boolean);

        if (!commandIds.length) {
          terminal.write("ERR HEATER_CONFIG; could not build command", "error");
          return;
        }

        commandIds.reduce(function (chain, commandId) {
          return chain.then(function () {
            return sendCommand(commandId, "button");
          });
        }, Promise.resolve()).catch(function () {
          return undefined;
        });
      });
    }
  }

  function bindViewSwitch() {
    dom.telemetryViewButton.addEventListener("click", function () {
      setTelemetryPanelView("telemetry");
    });

    dom.overviewViewButton.addEventListener("click", function () {
      setTelemetryPanelView("overview");
    });
  }

  function handleTerminalCommand(raw) {
    const normalized = normalizeCommand(raw);

    if (normalized === "help") {
    terminal.write("commands: status, clear, start experiment, enter standby, start/stop pressurisation, open/close outlet valve, k96 on/off, pwm1 0-100, pwm2 0-100, pwm3 0-100, relay 1-4 on/off, pump 1/2 on/off, compressor on/off, heater n on/off, heater all on/off, heater n mode pid|bangbang|manual, heater n target -100 - 200, heater n duty 0-100, heater n mode manual duty 0-100, enable/disable cooling, flush chamber, restart main controller, emergency stop");
      return;
    }

    if (normalized === "clear") {
      terminal.clear();
      terminal.write("terminal cleared");
      return;
    }

    if (normalized === "status") {
      terminal.write(formatStatusLine(latestTelemetry));
      return;
    }

    const pwmMatch = normalized.match(/^pwm([123])\s+(\d{1,3})$/);
    if (pwmMatch) {
      const percentage = Number(pwmMatch[2]);
      if (percentage > 100) {
        terminal.write("ERR PWM_VALUE; use a value from 0 to 100", "error");
        return;
      }
      sendCommand(registerPwmCommand(Number(pwmMatch[1]), percentage), "terminal").catch(function () {
        return undefined;
      });
      return;
    }

    const heaterCommandId = parseHeaterControlCommand(normalized);
    if (heaterCommandId) {
      const routeName = activeCommandRouter === gatewayCommandRouter ? "payload gateway" : "mock uplink";
      const definition = COMMANDS[heaterCommandId];
      if (definition && definition.disruptive && !window.confirm("Route '" + definition.label + "' through " + routeName + "?")) {
        terminal.write("manual command cancelled locally", "warn");
        return;
      }
      sendCommand(heaterCommandId, "terminal").catch(function () {
        return undefined;
      });
      return;
    }

    const commandId = commandAliases[normalized];
    if (!commandId) {
      terminal.write("ERR UNKNOWN_COMMAND; type help for available uplink commands", "error");
      log.add("warn", "Manual command rejected", raw + " is not mapped to an uplink command");
      return;
    }

    const routeName = activeCommandRouter === gatewayCommandRouter ? "payload gateway" : "mock uplink";
    if (COMMANDS[commandId].disruptive && !window.confirm("Route '" + COMMANDS[commandId].label + "' through " + routeName + "?")) {
      terminal.write("manual command cancelled locally", "warn");
      return;
    }

    sendCommand(commandId, "terminal").catch(function () {
      return undefined;
    });
  }

  function sendCommand(commandId, origin) {
    dom.commandStatus.textContent = "Uplink pending";
    return activeCommandRouter.send(commandId, origin).catch(function (error) {
      return Promise.reject(error);
    });
  }

  function handleCommandEvent(event) {
    if (event.type === "queued") {
      const route = activeCommandRouter === gatewayCommandRouter && event.wireCommand ? "; wire command " + event.wireCommand : "; mock delay " + event.delay + " ms";
      const message = event.label + " queued as " + event.requestId + " from " + event.origin + route;
      log.add("command", "Uplink queued", message);
      if (event.origin === "terminal") {
        terminal.write("QUEUED " + event.requestId + " " + event.label);
      }
      return;
    }

    if (event.type === "ack") {
      applyCommandedState(event.commandId);
      renderActuatorState(latestTelemetry);
      renderSystemOverview(latestTelemetry);
      dom.commandStatus.textContent = "ACK " + event.requestId;
      log.add("command", "Command acknowledged", event.requestId + " " + event.label + "; " + event.message);
      if (event.origin === "terminal") {
        terminal.write("ACK " + event.requestId + " " + event.message);
      }
      window.setTimeout(function () {
        dom.commandStatus.textContent = "Ready";
      }, 2600);
      return;
    }

    if (event.type === "nack") {
      dom.commandStatus.textContent = "NACK " + event.requestId;
      log.add("fault", "Command failed", event.requestId + " " + event.label + "; " + event.message);
      if (event.origin === "terminal") {
        terminal.write("NACK " + event.requestId + " " + event.message, "error");
      }
      window.setTimeout(function () {
        dom.commandStatus.textContent = "Ready";
      }, 3200);
    }
  }

  function handleFrame(sample) {
    history.push(sample);
    while (history.length > MAX_SAMPLES) {
      history.shift();
    }

    if (sample && typeof sample.mode === "string" && sample.mode) {
      latestMissionMode = sample.mode;
    }

    if (sample.valid) {
      latestTelemetry = sample;
      lastGoodFrameAt = sample.timestamp;
    }

    renderTelemetry(sample);
    drawAllCharts();
  }

  function renderTelemetry(sample) {
    const display = sample.valid ? sample : latestTelemetry;
    const health = sample.valid ? sample.health : "dropout";
    const linkStatus = sample.valid ? sample.linkStatus : "DROPOUT";
    const linkQuality = sample.valid ? sample.linkQuality : 0;

    setChip(dom.overallHealth, healthLabel(health), health);
    renderHealthDetails(sample && sample.valid ? sample.errors : []);
    setChip(dom.missionMode, resolveMissionMode(display), "neutral");
    setChip(dom.linkState, linkLabel(linkStatus), linkStatus === "ONLINE" ? "healthy" : linkStatus === "DEGRADED" ? "warning" : "dropout");

    dom.frameNumber.textContent = display && display.seq ? String(display.seq) : "--";
    dom.latencyValue.textContent = sample.valid ? sample.latencyMs + " ms" : "--";
    dom.storageState.textContent = legacyProtocol
      ? "Update required"
      : display && display.onboardLogging && Number.isFinite(display.storageFreePct)
        ? "SD " + display.storageFreePct.toFixed(0) + "% free"
        : "SD unavailable";
    dom.controllerState.textContent = controllerLabel(sample);
    dom.terminalRoute.textContent = activeCommandRouter === gatewayCommandRouter
      ? (linkStatus === "DROPOUT" ? "gateway route degraded" : "payload TCP route")
      : (linkStatus === "DROPOUT" ? "mock route degraded" : "local mock route");

    if (display) {
      const methaneRaw = Number.isFinite(display.methaneRaw) ? display.methaneRaw : null;
      dom.methaneValue.textContent = methaneRaw === null ? "--" : methaneRaw.toFixed(0);
      dom.chamberPressureValue.textContent = display.chamberPressureBar.toFixed(2);
      const chamberTemp = display?.chamberTempC_K96 ?? display?.chamberTempC ?? 0;
      dom.chamberTempValue.textContent = chamberTemp.toFixed(1);
      const humidityRH = display?.humidityRh_ambient ?? 0;
      dom.humidityValue.textContent = humidityRH.toFixed(0);
      dom.linkQualityValue.textContent = String(Math.round(linkQuality));

      setMetricState(dom.metricMethane, methaneRaw === null ? "dropout" : display.k96Error ? "warning" : "healthy");
      setMetricState(dom.metricPressure, pressureState(display.chamberPressureBar));
      setMetricState(dom.metricTemperature, temperatureState(display.chamberTempC_K96));
      setMetricState(dom.metricHumidity, humidityState(display.humidityRh_ambient));
      setMetricState(dom.metricLink, linkStatus === "DROPOUT" ? "dropout" : linkQuality < 75 ? "warning" : "healthy");
    }

    renderActuatorState(display);
    renderActiveTask(sample.valid ? sample : null);
    renderSystemOverview(display, health, linkStatus);
    updateFrameAge();
  }

  function updateFrameAge() {
    if (!lastGoodFrameAt) {
      dom.lastFrameAge.textContent = "--";
      return;
    }

    const seconds = Math.max(0, Math.round((Date.now() - lastGoodFrameAt) / 1000));
    dom.lastFrameAge.textContent = seconds === 0 ? "now" : seconds + " s ago";
  }

  function setTelemetryPanelView(view) {
    const showOverview = view === "overview";

    dom.telemetryView.hidden = showOverview;
    dom.systemOverviewView.hidden = !showOverview;
    dom.telemetryViewButton.classList.toggle("active", !showOverview);
    dom.overviewViewButton.classList.toggle("active", showOverview);
    dom.telemetryViewButton.setAttribute("aria-pressed", String(!showOverview));
    dom.overviewViewButton.setAttribute("aria-pressed", String(showOverview));

    if (showOverview) {
      renderSystemOverview(latestTelemetry);
    } else {
      window.requestAnimationFrame(drawAllCharts);
    }
  }

  function handleGatewayConnected() {
    if (usingGateway) {
      return;
    }

    usingGateway = true;
    activeCommandRouter = gatewayCommandRouter;
    if (MOCK_DATA_ENABLED) {
      mockTelemetrySource.stop();
    }
    history.length = 0;
    latestTelemetry = null;
    lastGoodFrameAt = 0;
    previousHealth = "unknown";
    previousLinkStatus = "unknown";
    terminal.write("E-Link gateway connected; telemetry is now decoded from the payload TCP status stream.");
    log.add("info", "Live gateway connected", MOCK_DATA_ENABLED ? "local Python gateway replaced mock telemetry and mock uplink routing" : "local Python gateway is now the only telemetry source");
  }

  function handleGatewayDisconnected() {
    if (!usingGateway) {
      return;
    }

    usingGateway = false;
    if (MOCK_DATA_ENABLED) {
      activeCommandRouter = mockCommandRouter;
      mockTelemetrySource.start();
      terminal.write("gateway disconnected; local mock route resumed", "warn");
    } else {
      activeCommandRouter = gatewayCommandRouter;
      terminal.write("gateway disconnected; telemetry halted until payload reconnects", "warn");
    }

    setChip(dom.linkState, "E-Link dropout", "dropout");
    setChip(dom.missionMode, resolveMissionMode(latestTelemetry), "neutral");
    dom.controllerState.textContent = "No telemetry";
    renderActiveTask(null);
  }

  function applyCommandedState(commandId) {
    const definition = COMMANDS[commandId];
    if (!definition) {
      return;
    }

    if (definition.stateTarget) {
      commandedState[definition.stateTarget.group][definition.stateTarget.key] = definition.stateTarget.value;
      return;
    }

    if (commandId === "startExperiment" || commandId === "startPressurisation") {
      setCommandedPressureTrain(true);
    }

    if (commandId === "flushChamber") {
      commandedState.peripherals.pump1 = true;
      commandedState.peripherals.pump2 = true;
      commandedState.peripherals.compressor = false;
    }

    if (commandId === "enterStandby" || commandId === "stopPressurisation" || commandId === "restartController") {
      setCommandedPressureTrain(false);
    }

    if (commandId === "openOutletValve" || commandId === "flushChamber" || commandId === "emergencyStop") {
      commandedState.peripherals.outletValve = true;
    }

    if (commandId === "closeOutletValve" || commandId === "startPressurisation" || commandId === "startExperiment") {
      commandedState.peripherals.outletValve = false;
    }

    if (commandId === "enableHeating") {
      commandedState.heaterMask |= DEFAULT_HEATER_MASK;
    }

    if (commandId === "disableHeating" || commandId === "emergencyStop") {
      commandedState.heaterMask &= ~DEFAULT_HEATER_MASK;
    }

    const heaterDefinition = definition.heaterBit === undefined ? null : definition;
    if (heaterDefinition) {
      if (heaterDefinition.heaterEnabled) {
        commandedState.heaterMask |= 1 << heaterDefinition.heaterBit;
      } else {
        commandedState.heaterMask &= ~(1 << heaterDefinition.heaterBit);
      }
    }

    if (commandId === "enableCooling") {
      commandedState.coolerMask = DEFAULT_COOLER_MASK;
    }

    if (commandId === "disableCooling" || commandId === "emergencyStop") {
      commandedState.coolerMask = 0x00;
    }

    if (commandId === "emergencyStop") {
      setCommandedPressureTrain(false);
    }
  }

  function setCommandedPressureTrain(enabled) {
    commandedState.peripherals.pump1 = Boolean(enabled);
    commandedState.peripherals.pump2 = Boolean(enabled);
    commandedState.peripherals.compressor = Boolean(enabled);
  }

  function renderActuatorState(sample) {
    const state = extractSystemState(sample);

    document.querySelectorAll("[data-toggle]").forEach(function (button) {
      const id = button.dataset.toggle;
      const active = Object.prototype.hasOwnProperty.call(state.relays, id) ? state.relays[id] : state.peripherals[id];
      const metadata = PRESSURE_PERIPHERALS.find(function (item) {
        return item.id === id;
      });
      const label = metadata ? (active ? metadata.onLabel : metadata.offLabel) : (active ? "ON" : "OFF");

      button.classList.toggle("is-active", Boolean(active));
      button.setAttribute("aria-pressed", String(Boolean(active)));

      const stateLabel = button.querySelector("small");
      if (stateLabel) {
        stateLabel.textContent = label;
      }
    });
  }

  function renderActiveTask(sample) {
    const activeTask = sample && sample.activeTask ? sample.activeTask : null;
    document.querySelectorAll("[data-task]").forEach(function (button) {
      const active = button.dataset.task === activeTask;
      button.classList.toggle("task-active", active);
      button.setAttribute("aria-busy", String(active));
    });
  }

  function renderSystemOverview(sample, healthOverride, linkOverride) {
    const state = extractSystemState(sample);
    const linkStatus = linkOverride || (sample ? sample.linkStatus : previousLinkStatus);
    const health = healthOverride || (sample ? sample.health : previousHealth);
    const pressureActive = state.peripherals.pump1 || state.peripherals.pump2 || state.peripherals.compressor || Boolean(sample && sample.pressureSystemOn);
    const heaterActive = (state.heaterMask & DEFAULT_HEATER_MASK) !== 0 || Boolean(sample && sample.heatingEnabled);
    const coolerActive = (state.heaterMask & DEFAULT_COOLER_MASK) !== 0 || state.coolerMask !== 0 || Boolean(sample && sample.coolingEnabled);
    const chamberState = sample ? worstState([
      pressureState(sample.chamberPressureBar),
      temperatureState(sample.chamberTempC_K96 ?? sample.chamberTempC_MS ?? sample.chamberTempC),
      humidityState(sample.humidityRh_ambient)
    ]) : "neutral";
    const linkState = linkStatus === "DROPOUT" ? "dropout" : linkStatus === "DEGRADED" ? "warn" : "on";
    const mainState = linkStatus === "DROPOUT" ? "dropout" : sample && sample.controllerReady === false ? "warn" : health === "fault" ? "fault" : health === "warning" ? "warn" : "on";
    const thermalState = sample && sample.thermalOnline === false ? "fault" : sample && sample.thermalError ? "warn" : "on";

    setDiagramNodeState("diagramElLink", linkState);
    setDiagramNodeState("diagramMainMcu", mainState);
    setDiagramNodeState("diagramPressureMcu", pressureActive ? "on" : "off");
    setDiagramNodeState("diagramThermalMcu", thermalState);
    setDiagramNodeState("diagramPreheater", heaterActive ? "on" : "off");
    setDiagramNodeState("diagramPump1", state.peripherals.pump1 ? "on" : "off");
    setDiagramNodeState("diagramPump2", state.peripherals.pump2 ? "on" : "off");
    setDiagramNodeState("diagramCompressor", state.peripherals.compressor ? "on" : "off");
    setDiagramNodeState("diagramChamber", chamberState === "healthy" ? "on" : chamberState);
    setDiagramNodeState("diagramOutletValve", state.peripherals.outletValve ? "on" : "off");
    setDiagramNodeState("diagramStorage", legacyProtocol || !sample || !sample.onboardLogging ? "fault" : sample.storageFreePct < 20 ? "warn" : "on");
    setDiagramNodeState("diagramHeaters", heaterActive ? "on" : "off");
    setDiagramNodeState("diagramCooler", coolerActive ? "on" : "off");

    setDiagramLineState("flowIntakePreheater", heaterActive ? "on" : "off");
    setDiagramLineState("flowPreheaterPump1", state.peripherals.pump1 ? "on" : "off");
    setDiagramLineState("flowPump1Pump2", state.peripherals.pump1 && state.peripherals.pump2 ? "on" : "off");
    setDiagramLineState("flowPump2Compressor", state.peripherals.pump2 && state.peripherals.compressor ? "on" : "off");
    setDiagramLineState("flowCompressorChamber", state.peripherals.compressor ? "on" : "off");
    setDiagramLineState("flowChamberValve", state.peripherals.outletValve ? "on" : "off");
    setDiagramLineState("flowValveOutlet", state.peripherals.outletValve ? "on" : "off");
    setDiagramLineState("busMainLink", linkState);
    setDiagramLineState("busMainPressure", pressureActive ? "on" : "off");
    setDiagramLineState("busPressureThermal", thermalState);
    setDiagramLineState("busThermalHeaters", heaterActive || coolerActive ? "on" : "off");
    setDiagramLineState("busMainStorage", "on");

    RELAY_LINES.forEach(function (relay, index) {
      const element = document.getElementById("diagramRelay" + (index + 1));
      if (element) {
        element.classList.toggle("is-on", Boolean(state.relays[relay.id]));
      }
      setText(dom["diagramRelay" + (index + 1) + "Value"], "R" + (index + 1) + " " + (state.relays[relay.id] ? "ON" : "OFF"));
    });

    setText(dom.diagramAmbientValue, sample ? sample.ambientPressureBar.toFixed(0) + " bar" : "-- bar");
    setText(dom.diagramPreheaterValue, heaterActive ? "ACTIVE" : "OFF");
    setText(dom.diagramPump1Value, state.peripherals.pump1 ? "ON" : "OFF");
    setText(dom.diagramPump2Value, state.peripherals.pump2 ? "ON" : "OFF");
    setText(dom.diagramCompressorValue, state.peripherals.compressor ? "ON" : "OFF");
    const chamberTemp = sample?.chamberTempC_K96 ?? sample?.chamberTempC ?? 0;
    setText(dom.diagramChamberValue, sample ? sample.chamberPressureBar.toFixed(2) + " bar / " + chamberTemp.toFixed(1) + " C" : "-- bar / -- C");
    setText(dom.diagramValveValue, state.peripherals.outletValve ? "OPEN" : "CLOSED");
    setText(dom.diagramLinkValue, linkLabel(linkStatus).replace("E-Link ", "").toUpperCase());
    setText(dom.diagramMainValue, controllerDiagramLabel(sample));
    setText(dom.diagramPressureMcuValue, sample && sample.activeTask ? sample.activeTask.replace("_", " ") : pressureActive ? "ACTIVE" : "STANDBY");
    setText(dom.diagramThermalMcuValue, sample && sample.thermalOnline === false ? "OFFLINE" : sample && sample.thermalError ? "WARN" : "ONLINE");
    setText(dom.diagramStorageValue, legacyProtocol ? "UPDATE" : sample && sample.onboardLogging && Number.isFinite(sample.storageFreePct) ? sample.storageFreePct.toFixed(0) + "% FREE" : "UNAVAILABLE");
    setText(dom.diagramHeaterValue, heaterActive ? describeMask(state.heaterMask & DEFAULT_HEATER_MASK) : "OFF");
    setText(dom.diagramCoolerValue, coolerActive ? "ACTIVE" : "OFF");
  }

  function extractSystemState(sample) {
    const relays = Object.assign({}, commandedState.relays);
    const peripherals = Object.assign({}, commandedState.peripherals);
    let heaterMask = commandedState.heaterMask | commandedState.coolerMask;

    if (sample) {
      if (sample.relayLines) {
        Object.assign(relays, sample.relayLines);
      }

      if (sample.peripherals) {
        Object.assign(peripherals, sample.peripherals);
      } else if (typeof sample.pressureSystemOn === "boolean") {
        peripherals.pump1 = sample.pressureSystemOn;
        peripherals.pump2 = sample.pressureSystemOn;
        peripherals.compressor = sample.pressureSystemOn;
      }

      if (typeof sample.outletValveOpen === "boolean") {
        peripherals.outletValve = sample.outletValveOpen;
      }

      if (typeof sample.heaterMask === "number") {
        heaterMask = sample.heaterMask | commandedState.coolerMask;
      }
    }

    return {
      relays: relays,
      peripherals: peripherals,
      heaterMask: heaterMask,
      coolerMask: commandedState.coolerMask
    };
  }

  function setDiagramNodeState(id, state) {
    const element = document.getElementById(id);
    if (!element) {
      return;
    }

    element.classList.remove("is-on", "is-off", "is-warn", "is-fault", "is-dropout");
    if (state === "on" || state === "healthy") {
      element.classList.add("is-on");
    } else if (state === "warn" || state === "warning") {
      element.classList.add("is-warn");
    } else if (state === "fault") {
      element.classList.add("is-fault");
    } else if (state === "dropout") {
      element.classList.add("is-dropout");
    } else {
      element.classList.add("is-off");
    }
  }

  function setDiagramLineState(id, state) {
    const element = document.getElementById(id);
    if (!element) {
      return;
    }

    element.classList.remove("is-on", "is-warn", "is-fault", "is-dropout");
    if (state === "on" || state === "healthy") {
      element.classList.add("is-on");
    } else if (state === "warn" || state === "warning") {
      element.classList.add("is-warn");
    } else if (state === "fault") {
      element.classList.add("is-fault");
    } else if (state === "dropout") {
      element.classList.add("is-dropout");
    }
  }

  function worstState(states) {
    if (states.indexOf("fault") !== -1) {
      return "fault";
    }
    if (states.indexOf("warning") !== -1) {
      return "warn";
    }
    if (states.indexOf("dropout") !== -1) {
      return "dropout";
    }
    return "healthy";
  }

  function describeMask(mask) {
    const active = THERMAL_CHANNELS
      .filter(function (channel) {
        return (mask & (1 << channel.bit)) !== 0;
      })
      .map(function (channel) {
        return "CH" + channel.bit;
      });

    return active.length ? active.join(",") : "OFF";
  }

  function setText(element, value) {
    if (element) {
      element.textContent = value;
    }
  }

  
  function drawAllCharts() {
    drawChart(dom.gasChart, history, {
      yLabel: "raw",
      series: [
        { key: "methaneRaw", color: "#61d394", min: 0, max: 65535 },
        { key: "co2Raw", color: "#7aa6ff", min: 0, max: 65535 },
        { key: "waterRaw", color: "#f0c15b", min: 0, max: 65535 }
      ]
    });

    drawChart(dom.pressureChart, history, {
      targetBand: { min: 2.85, max: 3.15, seriesMin: 2.2, seriesMax: 3.8 },
      yLabel: "bar",
      series: [
        { key: "Interstage_1Bar", color: getColorFromCssClass("interstage-1","background-color"), min: 0.01, max: 5.0 },
        { key: "Interstage_2Bar", color: getColorFromCssClass("interstage-2","background-color"), min: 0.01, max: 5.0 },
        { key: "chamberPressureBar", color: getColorFromCssClass("chamber-pressure","background-color"), min: 0.01, max: 5.0 },
      ]
    });

    drawChart(dom.thermalChart, history, {
      targetBand: { min: 19, max: 24, seriesMin: -60, seriesMax: 80 },
      yLabel: "deg C",
      series: [
        { key: "sdCardC", color: getColorFromCssClass("SD-temp","background-color"), min: -60, max: 80 },
        { key: "pump1C", color: getColorFromCssClass("pump1-temp","background-color"), min: -60, max: 70 },
        { key: "pump2C", color: getColorFromCssClass("pump2-temp","background-color"), min: -60, max: 70 },
        { key: "compressorC", color: getColorFromCssClass("pump3-temp","background-color"), min: -60, max: 70 },
        { key: "Interstage1_C", color: getColorFromCssClass("interstage1-temp","background-color"), min: -60, max: 70 },
        { key: "Interstage2_C", color: getColorFromCssClass("interstage2-temp","background-color"), min: -60, max: 70 },
        { key: "chamberTempC_MS", color: getColorFromCssClass("chamber-tempMS","background-color"), min: -60, max: 80 },
        { key: "chamberTempC_K96", color: getColorFromCssClass("chamber-tempK96","background-color"), min: -60, max: 70 },
      ]
    });

    drawChart(dom.ambientChart, history, {
      yLabel: "bar/C/%",
      series: [
        { key: "ambientPressureBar", color: getColorFromCssClass("ambient-pressure","background-color"), min: 0, max: 2 },
        { key: "ambientTempC_TMP", color: getColorFromCssClass("temperature-tmp117","background-color"), min: -60, max: 80 },
        { key: "ambientTempC_SHT", color: getColorFromCssClass("temperature-sht45","background-color"), min: -60, max: 80 },
        { key: "ambientTempC_MS", color: getColorFromCssClass("temperature-ms5803","background-color"), min: -60, max: 80 },
        { key: "humidityRh_ambient", color: getColorFromCssClass("ambient-humidity","background-color"), min: 0, max: 100 },
        { key: "humidityRh_k96", color: getColorFromCssClass("humidity-k96","background-color"), min: 0, max: 100 }
      ]
    });

    drawChart(dom.linkChart, history, {
      yLabel: "%",
      series: [
        { key: "linkQuality", color: "#61d394", min: 0, max: 100 },
        { key: "pump1DutyPct", color: "#f0c15b", min: 0, max: 100 },
        { key: "pump2DutyPct", color: "#7aa6ff", min: 0, max: 100 },
        { key: "compressorDutyPct", color: "#ff9f57", min: 0, max: 100 }
      ]
    });

    drawChart(dom.heaterActuationChart, history, {
      yLabel: "%",
      series: [
        { key: "heater1ActuationPct", color: "#ff6b68", min: 0, max: 100 },
        { key: "heater2ActuationPct", color: "#f0c15b", min: 0, max: 100 },
        { key: "heater3ActuationPct", color: "#61d394", min: 0, max: 100 },
        { key: "heater4ActuationPct", color: "#73fff6", min: 0, max: 100 },
        { key: "heater5ActuationPct", color: "#7aa6ff", min: 0, max: 100 },
        { key: "heater6ActuationPct", color: "#b589ff", min: 0, max: 100 },
        { key: "heater7ActuationPct", color: "#ff9f57", min: 0, max: 100 },
        { key: "heater8ActuationPct", color: "#e8eef2", min: 0, max: 100 }
      ]
    });


  }

  function getColorFromCssClass(className, cssProperty = "color") {
  if (!className) return "#ffffff";
  
  // Temporary hidden element to query stylesheet rules
  const tempEl = document.createElement("div");
  tempEl.className = className;
  tempEl.style.display = "none";
  document.body.appendChild(tempEl);

  const computedColor = getComputedStyle(tempEl).getPropertyValue(cssProperty);
  document.body.removeChild(tempEl);

  return computedColor || "#ffffff";
}

  function drawChart(canvas, samples, config) {
  if (!canvas) {
    return;
  }

  // 1. STORE RECENT DATA & BIND MOUSE LISTENERS (ONCE)
  canvas._lastSamples = samples;
  canvas._lastConfig = config;

  if (!canvas._hoverListenersBound) {
    canvas._hoverListenersBound = true;

    canvas.addEventListener("mousemove", function (e) {
      const rect = canvas.getBoundingClientRect();
      canvas._hoverPos = {
        x: e.clientX - rect.left,
        y: e.clientY - rect.top
      };
      if (canvas._lastSamples && canvas._lastConfig) {
        drawChart(canvas, canvas._lastSamples, canvas._lastConfig);
      }
    });

    canvas.addEventListener("mouseleave", function () {
      canvas._hoverPos = null;
      if (canvas._lastSamples && canvas._lastConfig) {
        drawChart(canvas, canvas._lastSamples, canvas._lastConfig);
      }
    });
  }

  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(260, Math.floor(rect.width));
  const height = Math.max(110, Math.floor(rect.height));
  const pixelWidth = Math.floor(width * dpr);
  const pixelHeight = Math.floor(height * dpr);

  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }

  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const hasYLabel = Boolean(config && config.yLabel);
  const pad = { 
    left: hasYLabel ? 62 : 46, 
    top: 12, 
    right: 12, 
    bottom: 22 
  };

  const plotW = width - pad.left - pad.right;
  const plotH = height - pad.top - pad.bottom;

  ctx.fillStyle = "#111511";
  ctx.fillRect(0, 0, width, height);

  const yRange = getChartYRange(config, samples);

  drawGrid(ctx, pad, plotW, plotH);
  drawDropouts(ctx, samples, pad, plotW, plotH);

  if (config.targetBand) {
    const y1 = yFor(config.targetBand.max, yRange.min, yRange.max, pad, plotH);
    const y2 = yFor(config.targetBand.min, yRange.min, yRange.max, pad, plotH);
    ctx.fillStyle = "rgba(97, 211, 148, 0.09)";
    ctx.fillRect(pad.left, y1, plotW, y2 - y1);
    ctx.strokeStyle = "rgba(97, 211, 148, 0.28)";
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(pad.left, y1);
    ctx.lineTo(pad.left + plotW, y1);
    ctx.moveTo(pad.left, y2);
    ctx.lineTo(pad.left + plotW, y2);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  config.series.forEach(function (series) {
    ctx.strokeStyle = series.color;
    ctx.lineWidth = 2;
    ctx.beginPath();

    let started = false;
    samples.forEach(function (sample, index) {
      const value = sample && sample.valid ? sample[series.key] : null;
      if (typeof value !== "number" || Number.isNaN(value)) {
        started = false;
        return;
      }

      const x = pad.left + xFor(index, samples.length, plotW);
      const y = yFor(value, yRange.min, yRange.max, pad, plotH);

      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    });

    ctx.stroke();
  });

  drawAxes(ctx, samples, pad, plotW, plotH, config, yRange);

  // 2. RENDER HOVER OVERLAY (CROSSHAIR & TOOLTIP)
  if (canvas._hoverPos && samples && samples.length > 0) {
    drawTooltip(ctx, canvas._hoverPos, samples, pad, plotW, plotH, config, yRange, width);
  }
}

// 3. NEW HELPER FUNCTION TO DRAW HOVER CROSSHAIR AND FLOATING CARD
function drawTooltip(ctx, hoverPos, samples, pad, plotW, plotH, config, yRange, canvasWidth) {
  // Ignore hover if cursor is outside the plotting area
  if (hoverPos.x < pad.left || hoverPos.x > pad.left + plotW) {
    return;
  }

  // Calculate nearest sample index based on cursor X
  const ratio = (hoverPos.x - pad.left) / plotW;
  const rawIndex = Math.round(ratio * (samples.length - 1));
  const index = Math.max(0, Math.min(samples.length - 1, rawIndex));
  const sample = samples[index];

  if (!sample) return;

  const sampleX = pad.left + xFor(index, samples.length, plotW);

  // Draw Vertical Crosshair Line
  ctx.save();
  ctx.strokeStyle = "rgba(255, 255, 255, 0.4)";
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.moveTo(sampleX, pad.top);
  ctx.lineTo(sampleX, pad.top + plotH);
  ctx.stroke();
  ctx.restore();

  // Draw Highlight Dots on Active Series Points
  const activePoints = [];
  config.series.forEach(function (series) {
    const val = sample.valid ? sample[series.key] : null;
    if (typeof val === "number" && !Number.isNaN(val)) {
      const ptY = yFor(val, yRange.min, yRange.max, pad, plotH);
      
      // Outer ring
      ctx.fillStyle = "#111511";
      ctx.beginPath();
      ctx.arc(sampleX, ptY, 5, 0, Math.PI * 2);
      ctx.fill();

      // Colored core
      ctx.fillStyle = series.color;
      ctx.beginPath();
      ctx.arc(sampleX, ptY, 3, 0, Math.PI * 2);
      ctx.fill();

      activePoints.push({
        label: series.label || series.key,
        color: series.color,
        value: formatYAxisLabel(val)
      });
    }
  });

  // Calculate Time Offset Label (e.g. "-12s" or "now")
  const secondsAgo = samples.length - 1 - index;
  const timeText = secondsAgo === 0 ? "now" : "-" + secondsAgo + "s";

  // Prepare Tooltip Content Lines
  const lines = [timeText];
  if (!sample.valid) {
    lines.push("Status: DROPOUT");
  } else {
    activePoints.forEach(pt => lines.push(pt.label + ": " + pt.value));
  }

  // Measure Tooltip Box Dimensions
  ctx.font = "11px Consolas, monospace";
  let boxW = 0;
  lines.forEach(line => {
    boxW = Math.max(boxW, ctx.measureText(line).width);
  });
  boxW += 16; // Internal padding
  const lineH = 15;
  const boxH = lines.length * lineH + 10;

  // Position Tooltip Card (Flip to left if too close to right edge)
  let boxX = sampleX + 12;
  if (boxX + boxW > canvasWidth - 10) {
    boxX = sampleX - boxW - 12;
  }
  let boxY = pad.top + 5;

  // Draw Tooltip Card Background & Border
  ctx.fillStyle = "rgba(18, 22, 18, 0.92)";
  ctx.strokeStyle = "rgba(174, 184, 167, 0.35)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect ? ctx.roundRect(boxX, boxY, boxW, boxH, 4) : ctx.rect(boxX, boxY, boxW, boxH);
  ctx.fill();
  ctx.stroke();

  // Render Tooltip Text Lines
  ctx.textAlign = "left";
  ctx.textBaseline = "top";

  // Title / Time
  ctx.fillStyle = "rgba(174, 184, 167, 0.65)";
  ctx.fillText(lines[0], boxX + 8, boxY + 6);

  // Series values
  let currentY = boxY + 6 + lineH;
  if (!sample.valid) {
    ctx.fillStyle = "#b589ff";
    ctx.fillText(lines[1], boxX + 8, currentY);
  } else {
    activePoints.forEach(pt => {
      ctx.fillStyle = pt.color;
      ctx.fillText(pt.label + ": " + pt.value, boxX + 8, currentY);
      currentY += lineH;
    });
  }
}

  function drawGrid(ctx, pad, plotW, plotH) {
    ctx.strokeStyle = "rgba(174, 184, 167, 0.12)";
    ctx.lineWidth = 1;

    for (let i = 0; i <= 4; i += 1) {
      const y = pad.top + (plotH / 4) * i;
      ctx.beginPath();
      ctx.moveTo(pad.left, y);
      ctx.lineTo(pad.left + plotW, y);
      ctx.stroke();
    }

    for (let i = 0; i <= 5; i += 1) {
      const x = pad.left + (plotW / 5) * i;
      ctx.beginPath();
      ctx.moveTo(x, pad.top);
      ctx.lineTo(x, pad.top + plotH);
      ctx.stroke();
    }
  }

  function drawDropouts(ctx, samples, pad, plotW, plotH) {
    if (samples.length < 2) {
      return;
    }

    ctx.fillStyle = "rgba(181, 137, 255, 0.12)";
    samples.forEach(function (sample, index) {
      if (sample.valid) {
        return;
      }
      const x = pad.left + xFor(index, samples.length, plotW);
      ctx.fillRect(x - 2, pad.top, 4, plotH);
    });
  }

  function drawAxes(ctx, samples, pad, plotW, plotH, config, yRange) {
  ctx.strokeStyle = "rgba(174, 184, 167, 0.26)";
  ctx.lineWidth = 1;
  ctx.strokeRect(pad.left, pad.top, plotW, plotH);

  ctx.fillStyle = "rgba(174, 184, 167, 0.72)";
  ctx.font = "11px Consolas, monospace";

  // Use pre-computed yRange instead of recalculating
  const tickValues = buildYAxisTicks(yRange.min, yRange.max, 5);

  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  const tickLabelX = pad.left - 7; 

  tickValues.forEach(function (tickValue) {
    const y = yFor(tickValue, yRange.min, yRange.max, pad, plotH);
    const label = formatYAxisLabel(tickValue);

    ctx.strokeStyle = "rgba(174, 184, 167, 0.18)";
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(pad.left + plotW, y);
    ctx.stroke();

    ctx.strokeStyle = "rgba(174, 184, 167, 0.26)";
    ctx.beginPath();
    ctx.moveTo(pad.left, y);
    ctx.lineTo(pad.left - 4, y);
    ctx.stroke();

    ctx.fillText(label, tickLabelX, y);
  });

  if (config && config.yLabel) {
    ctx.save();
    ctx.fillStyle = "rgba(174, 184, 167, 0.78)";
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.translate(13, pad.top + plotH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText(config.yLabel, 0, 0);
    ctx.restore();
  }

  ctx.textBaseline = "top";
  ctx.textAlign = "left";
  ctx.fillText("-" + Math.min(samples.length, MAX_SAMPLES) + "s", pad.left, pad.top + plotH + 5);

  ctx.textAlign = "right";
  ctx.fillText("now", pad.left + plotW, pad.top + plotH + 5);
}

  function getChartYRange(config, samples) {
  let minVal = Infinity;
  let maxVal = -Infinity;

  // 1. Scan all valid samples across all active series
  if (samples && samples.length && config && config.series) {
    samples.forEach(function (sample) {
      if (!sample || !sample.valid) return;

      config.series.forEach(function (series) {
        const value = sample[series.key];
        if (typeof value === "number" && !Number.isNaN(value)) {
          if (value < minVal) minVal = value;
          if (value > maxVal) maxVal = value;
        }
      });
    });
  }

  // Optional: Ensure target band fits inside the view if present
  if (config && config.targetBand) {
    if (typeof config.targetBand.min === "number") minVal = Math.min(minVal, config.targetBand.min);
    if (typeof config.targetBand.max === "number") maxVal = Math.max(maxVal, config.targetBand.max);
  }

  // 2. Fallback if no valid sample data is currently present
  if (minVal === Infinity || maxVal === -Infinity) {
    const firstSeries = config && config.series && config.series[0];
    return {
      min: firstSeries ? firstSeries.min : 0,
      max: firstSeries ? firstSeries.max : 1
    };
  }

  // 3. Flatline handling (e.g. constant value where min === max)
  if (minVal === maxVal) {
    const delta = Math.abs(minVal) * 0.1 || 1; // 10% offset or ±1 unit
    return {
      min: minVal - delta,
      max: maxVal + delta
    };
  }

  // 4. Add 8% padding (headroom & footroom) so line graphs don't touch the borders
  const range = maxVal - minVal;
  const padding = range * 0.08;

  return {
    min: minVal - padding,
    max: maxVal + padding
  };
}

  function buildYAxisTicks(min, max, count) {
    if (!Number.isFinite(min) || !Number.isFinite(max) || count < 2) {
      return [min, max];
    }

    const ticks = [];
    const step = (max - min) / (count - 1);

    for (let index = 0; index < count; index += 1) {
      ticks.push(min + step * index);
    }

    return ticks;
  }

  function formatYAxisLabel(value) {
    const absValue = Math.abs(value);

    if (absValue >= 100) {
      return value.toFixed(0);
    }

    if (absValue >= 10) {
      return value.toFixed(1);
    }

    return value.toFixed(2);
  }

  function xFor(index, length, plotW) {
    if (length <= 1) {
      return plotW;
    }
    return (index / Math.max(1, length - 1)) * plotW;
  }

  function yFor(value, min, max, pad, plotH) {
    const normalized = clamp((value - min) / (max - min), 0, 1);
    return pad.top + plotH - normalized * plotH;
  }

  function setChip(element, text, state) {
    element.textContent = text;
    element.classList.remove("chip-ok", "chip-warn", "chip-fault", "chip-dropout");

    if (state === "healthy") {
      element.classList.add("chip-ok");
    } else if (state === "warning") {
      element.classList.add("chip-warn");
    } else if (state === "fault") {
      element.classList.add("chip-fault");
    } else if (state === "dropout") {
      element.classList.add("chip-dropout");
    }
  }

  function renderHealthDetails(errors) {
    const detected = Array.isArray(errors) ? errors : [];
    dom.healthDetails.innerHTML = "<strong>Detected errors</strong>";
    if (!detected.length) {
      dom.healthDetails.insertAdjacentHTML("beforeend", '<span class="health-empty">No captured errors in this loop</span>');
      return;
    }

    const list = document.createElement("ul");
    detected.forEach(function (error) {
      const item = document.createElement("li");
      item.textContent = "Bit " + error.bit + ": " + error.message;
      list.appendChild(item);
    });
    dom.healthDetails.appendChild(list);
  }

  function setMetricState(element, state) {
    element.classList.remove("warn", "fault", "dropout");
    if (state === "warning") {
      element.classList.add("warn");
    } else if (state === "fault") {
      element.classList.add("fault");
    } else if (state === "dropout") {
      element.classList.add("dropout");
    }
  }

  function pressureState(value) {
    if (value < 2.35 || value > 3.55) {
      return "fault";
    }
    if (value < 2.74 || value > 3.22) {
      return "warning";
    }
    return "healthy";
  }

  function temperatureState(value) {
    if (value < 5 || value > 50) {
      return "fault";
    }
    if (value < 15 || value > 40) {
      return "warning";
    }
    return "healthy";
  }

  function humidityState(value) {
    if (value > 84) {
      return "fault";
    }
    if (value > 65) {
      return "warning";
    }
    return "healthy";
  }

  function healthLabel(health) {
    if (health === "healthy") {
      return "Healthy";
    }
    if (health === "warning") {
      return "Warning";
    }
    if (health === "fault") {
      return "Fault";
    }
    return "Dropout";
  }

  function linkLabel(status) {
    if (status === "ONLINE") {
      return "E-Link online";
    }
    if (status === "DEGRADED") {
      return "E-Link degraded";
    }
    return "E-Link dropout";
  }

  function controllerLabel(sample) {
    if (!sample || !sample.valid || sample.controller === "LINK_DROP") {
      return "No telemetry";
    }
    if (legacyProtocol) {
      return "Update required";
    }
    const labels = {
      MAIN_MCU_BOOTING: "Main MCU booting",
      MAIN_MCU_READY: "Main MCU ready",
      MAIN_MCU_SAFE_SHUTDOWN: "Main MCU safe",
      MAIN_MCU_RESTARTING: "Main MCU restarting"
    };
    return labels[sample.controller] || "Main MCU state unknown";
  }

  function controllerDiagramLabel(sample) {
    if (!sample) {
      return "NO DATA";
    }
    if (legacyProtocol) {
      return "UPDATE";
    }
    const labels = {
      MAIN_MCU_BOOTING: "BOOTING",
      MAIN_MCU_READY: "READY",
      MAIN_MCU_SAFE_SHUTDOWN: "SAFE",
      MAIN_MCU_RESTARTING: "RESTARTING",
      LINK_DROP: "NO LINK"
    };
    return labels[sample.controller] || "UNKNOWN";
  }

  function resolveMissionMode(sample) {
    if (sample) {
      if (typeof sample.missionMode === "string" && sample.missionMode) {
        return sample.missionMode;
      }

      if (typeof sample.mode === "string" && sample.mode) {
        return sample.mode;
      }

      if (typeof sample.mode === "number" && Number.isFinite(sample.mode)) {
        return "MODE_" + sample.mode;
      }
    }

    return latestMissionMode || "No telemetry";
  }

  function formatStatusLine(sample) {
    if (!sample) {
      return "NO_VALID_TELEMETRY";
    }

    const rawValue = function (value) {
      return Number.isFinite(value) ? value.toFixed(0) : "unavailable";
    };
    return [
      "MODE=" + sample.mode,
      "HEALTH=" + sample.health.toUpperCase(),
      "CH4_RAW=" + rawValue(sample.methaneRaw),
      "CO2_RAW=" + rawValue(sample.co2Raw),
      "H2O_RAW=" + rawValue(sample.waterRaw),
      "P_CHAMBER=" + sample.chamberPressureBar.toFixed(2) + "bar",
      "T_CHAMBER=" + (sample.chamberTempC_K96 ?? sample.chamberTempC ?? 0).toFixed(1) + "C",
      "RH=" + (sample.humidityRh_ambient ?? 0).toFixed(0) + "%",
      "LINK=" + sample.linkStatus
    ].join(" ");
  }

  function canUseGateway() {
    return window.location.protocol === "http:" || window.location.protocol === "https:";
  }

  function buildCommandAliases(commands) {
    const aliases = {};
    Object.keys(commands).forEach(function (commandId) {
      commands[commandId].aliases.forEach(function (alias) {
        aliases[normalizeCommand(alias)] = commandId;
      });
    });
    return aliases;
  }

  function normalizeCommand(value) {
    return value
      .toLowerCase()
      .replace(/[_-]+/g, " ")
      .replace(/\s+/g, " ")
      .trim();
  }

  function registerHeaterControlCommand(commandId, label, wireCommand, effect, aliases) {
    if (!COMMANDS[commandId]) {
      COMMANDS[commandId] = {
        label: label,
        wireCommand: wireCommand,
        aliases: aliases || [],
        effect: effect
      };
    }
    return commandId;
  }

  function parseHeaterControlCommand(normalized) {
    const allMatch = normalized.match(/^heater\s+all\s+(on|off)$/);
    if (allMatch) {
      const commandId = allMatch[1] === "on" ? "enableHeating" : "disableHeating";
      return commandId;
    }

    const heaterToggleMatch = normalized.match(/^heater\s+(?:on|off)\s+(\d+)$/);
    if (heaterToggleMatch) {
      const heaterNumber = Number(heaterToggleMatch[1]);
      const action = normalized.split(" ")[1];
      const commandId = "heater" + heaterNumber + (action === "on" ? "On" : "Off");
      return commandId;
    }

    const heaterSingleToggleMatch = normalized.match(/^heater\s+(\d+)\s+(on|off)$/);
    if (heaterSingleToggleMatch) {
      const heaterNumber = Number(heaterSingleToggleMatch[1]);
      const action = heaterSingleToggleMatch[2];
      const commandId = "heater" + heaterNumber + (action === "on" ? "On" : "Off");
      return commandId;
    }

    const heaterConfigMatch = normalized.match(/^heater\s+(\d+)\s+mode\s+(bangbang|bang-bang|pid|manual)(?:\s+target\s+([-+]?\d*\.?\d+))?(?:\s+duty\s+(\d{1,3}))?$/);
    if (heaterConfigMatch) {
      const heaterNumber = Number(heaterConfigMatch[1]);
      const mode = heaterConfigMatch[2] === "bang-bang" ? "BANGBANG" : heaterConfigMatch[2].toUpperCase();
      const target = heaterConfigMatch[3] !== undefined ? Number(heaterConfigMatch[3]) : undefined;
      const duty = heaterConfigMatch[4] !== undefined ? Number(heaterConfigMatch[4]) : undefined;
      const commandId = "heaterConfig_" + heaterNumber + "_" + mode + "_" + (target !== undefined ? String(target) : "") + "_" + (duty !== undefined ? String(duty) : "");
      const wireCommandParts = ["HEATER", String(heaterNumber), "MODE", mode];
      if (target !== undefined) {
        wireCommandParts.push("TARGET", String(target));
      }
      if (duty !== undefined) {
        wireCommandParts.push("DUTY", String(duty));
      }
      const wireCommand = wireCommandParts.join(" ");
      const label = "configure heater " + heaterNumber + " mode " + mode + (target !== undefined ? " target " + String(target) : "") + (duty !== undefined ? " duty " + String(duty) : "");

      registerHeaterControlCommand(commandId, label, wireCommand, function (sim) {
        if (!sim.heaterSettings) {
          sim.heaterSettings = {};
        }
        if (mode === "MANUAL") {
          sim.heaterSettings[heaterNumber] = { mode: mode, duty: duty !== undefined ? duty : 0 };
        } else {
          sim.heaterSettings[heaterNumber] = { mode: mode, target: target !== undefined ? target : undefined };
        }
        if (duty !== undefined && duty > 0 && mode === "MANUAL") {
          sim.heaterMask |= 1 << (heaterNumber - 1);
        }
        return "heater " + heaterNumber + " configured for " + mode + (target !== undefined ? " at " + target + "C" : "") + (duty !== undefined ? " at " + duty + "% duty" : "");
      }, ["heater " + heaterNumber + " mode " + mode.toLowerCase() + (target !== undefined ? " target " + String(target) : "") + (duty !== undefined ? " duty " + String(duty) : "")]);
      return commandId;
    }

    const heaterTargetMatch = normalized.match(/^heater\s+(\d+)\s+target\s+([-+]?\d*\.?\d+)$/);
    if (heaterTargetMatch) {
      const heaterNumber = Number(heaterTargetMatch[1]);
      const target = Number(heaterTargetMatch[2]);
      const commandId = "heaterTarget_" + heaterNumber + "_" + String(target);
      const wireCommand = "HEATER " + heaterNumber + " TARGET " + String(target);
      registerHeaterControlCommand(commandId, "set heater " + heaterNumber + " target to " + target + "C", wireCommand, function (sim) {
        if (!sim.heaterSettings) {
          sim.heaterSettings = {};
        }
        const settings = sim.heaterSettings[heaterNumber] || {};
        settings.target = target;
        sim.heaterSettings[heaterNumber] = settings;
        return "heater " + heaterNumber + " target set to " + target + "C";
      }, ["heater " + heaterNumber + " target " + String(target)]);
      return commandId;
    }

    const heaterDutyMatch = normalized.match(/^heater\s+(\d+)\s+duty\s+(\d{1,3})$/);
    if (heaterDutyMatch) {
      const heaterNumber = Number(heaterDutyMatch[1]);
      const duty = Number(heaterDutyMatch[2]);
      const commandId = "heaterDuty_" + heaterNumber + "_" + String(duty);
      const wireCommand = "HEATER " + heaterNumber + " DUTY " + String(duty);
      registerHeaterControlCommand(commandId, "set heater " + heaterNumber + " manual duty to " + duty + "%", wireCommand, function (sim) {
        if (!sim.heaterSettings) {
          sim.heaterSettings = {};
        }
        const settings = sim.heaterSettings[heaterNumber] || {};
        settings.mode = "MANUAL";
        settings.duty = duty;
        sim.heaterSettings[heaterNumber] = settings;
        if (duty > 0) {
          sim.heaterMask |= 1 << (heaterNumber - 1);
        }
        return "heater " + heaterNumber + " manual duty set to " + duty + "%";
      }, ["heater " + heaterNumber + " duty " + String(duty)]);
      return commandId;
    }

    const heaterModeMatch = normalized.match(/^heater\s+(\d+)\s+mode\s+(bangbang|bang-bang|pid|manual)$/);
    if (heaterModeMatch) {
      const heaterNumber = Number(heaterModeMatch[1]);
      const mode = heaterModeMatch[2] === "bang-bang" ? "BANGBANG" : heaterModeMatch[2].toUpperCase();
      const commandId = "heaterMode_" + heaterNumber + "_" + mode;
      const wireCommand = "HEATER " + heaterNumber + " MODE " + mode;
      registerHeaterControlCommand(commandId, "set heater " + heaterNumber + " mode " + mode, wireCommand, function (sim) {
        if (!sim.heaterSettings) {
          sim.heaterSettings = {};
        }
        const settings = sim.heaterSettings[heaterNumber] || {};
        settings.mode = mode;
        sim.heaterSettings[heaterNumber] = settings;
        return "heater " + heaterNumber + " mode set to " + mode;
      }, ["heater " + heaterNumber + " mode " + mode.toLowerCase()]);
      return commandId;
    }

    return null;
  }

  function registerPwmCommand(pump, percentage) {
    const commandId = "pwm" + pump + "_" + percentage;
    if (!COMMANDS[commandId]) {
      const label = "set pump " + pump + " PWM to " + percentage + "%";
      COMMANDS[commandId] = {
        label: label,
        wireCommand: "PWM" + pump + " " + percentage,
        aliases: [],
        effect: function (sim) {
          sim.setPeripheral(pump === 3 ? "compressor" : "pump" + pump, percentage > 0);
          return label + " queued";
        }
      };
    }
    return commandId;
  }

  function formatClock(date) {
    return date.toLocaleTimeString([], {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit"
    });
  }

  function approach(current, target, factor) {
    return current + (target - current) * factor;
  }

  function noise(amount) {
    return (Math.random() - 0.5) * 2 * amount;
  }

  function randInt(min, max) {
    return Math.floor(min + Math.random() * (max - min + 1));
  }

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  /**
 * Attaches vertical top-drag resizing logic to a bottom panel
 * @param {string} handleId - Element ID of the resize handle bar
 */
function makePanelResizable(handleId) {
  const handle = document.getElementById(handleId);
  const panel = handle?.closest(".panel");

  if (!handle || !panel) return;

  let startY = 0;
  let startHeight = 0;

  const onMouseMove = (e) => {
    const deltaY = e.clientY - startY;
    const newHeight = startHeight - deltaY;

    // Dynamics limit: prevents collapsing top panel completely (reserves 140px for top header)
    const parentColumn = panel.parentElement;
    const maxAllowedHeight = parentColumn ? parentColumn.clientHeight - 140 : 600;

    if (newHeight >= 120 && newHeight <= maxAllowedHeight) {
      panel.style.height = `${newHeight}px`;
    }
  };

  const onMouseUp = () => {
    panel.classList.remove("is-resizing");
    document.body.style.userSelect = "";
    document.removeEventListener("mousemove", onMouseMove);
    document.removeEventListener("mouseup", onMouseUp);
  };

  handle.addEventListener("mousedown", (e) => {
    e.preventDefault();
    startY = e.clientY;
    startHeight = panel.offsetHeight;

    panel.classList.add("is-resizing");
    document.body.style.userSelect = "none";

    document.addEventListener("mousemove", onMouseMove);
    document.addEventListener("mouseup", onMouseUp);
  });
}

function makeColumnsResizable() {
  const grid = document.querySelector(".console-grid");
  const handle = document.getElementById("columnResizeHandle");

  if (!grid || !handle) return;

  const minLeft = 330;
  const minRight = 420;
  let startX = 0;
  let startLeft = 0;

  const setColumns = (left, right) => {
    grid.style.setProperty("--left-column-width", `${left}px`);
    grid.style.setProperty("--right-column-width", `${right}px`);
    handle.setAttribute("aria-valuenow", String(Math.round(left)));
  };

  const onMouseMove = (event) => {
    const delta = event.clientX - startX;
    const availableWidth = grid.clientWidth - 12 - 10;
    const left = Math.max(minLeft, Math.min(availableWidth - minRight, startLeft + delta));
    const right = Math.max(minRight, availableWidth - left);
    setColumns(left, right);
  };

  const stopResizing = () => {
    grid.classList.remove("is-resizing-columns");
    document.body.style.userSelect = "";
    document.removeEventListener("mousemove", onMouseMove);
    document.removeEventListener("mouseup", stopResizing);
  };

  handle.addEventListener("mousedown", (event) => {
    event.preventDefault();
    const columns = grid.querySelectorAll(":scope > .console-column");
    startX = event.clientX;
    startLeft = columns[0].offsetWidth;
    grid.classList.add("is-resizing-columns");
    document.body.style.userSelect = "none";
    document.addEventListener("mousemove", onMouseMove);
    document.addEventListener("mouseup", stopResizing);
  });

  handle.setAttribute("aria-valuemin", String(minLeft));
  handle.setAttribute("aria-valuemax", String(Math.max(minLeft, grid.clientWidth - 12 - 10 - minRight)));
  handle.setAttribute("aria-valuenow", String(Math.round(grid.querySelector(":scope > .console-column").offsetWidth)));

  handle.addEventListener("keydown", (event) => {
    if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
    event.preventDefault();
    const columns = grid.querySelectorAll(":scope > .console-column");
    const currentLeft = columns[0].offsetWidth;
    const availableWidth = grid.clientWidth - 12 - 10;
    const nextLeft = currentLeft + (event.key === "ArrowRight" ? 20 : -20);
    const left = Math.max(minLeft, Math.min(availableWidth - minRight, nextLeft));
    setColumns(left, Math.max(minRight, availableWidth - left));
  });
}

// Initialize resizers once DOM is fully loaded
document.addEventListener("DOMContentLoaded", () => {
  makePanelResizable("commandResizeHandle");
  makePanelResizable("terminalResizeHandle");
  makeColumnsResizable();
});

  init();
})();
