const assert = require('assert');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const repoRoot = path.resolve(__dirname, '..');
const htmlPath = path.join(repoRoot, 'tools', 'EXTERNAL_IMU_UART_MONITOR.html');
const html = fs.readFileSync(htmlPath, 'utf8');
const match = html.match(
  /\/\/ PROTOCOL_TEST_EXPORT_START([\s\S]*?)\/\/ PROTOCOL_TEST_EXPORT_END/
);

assert(match, 'protocol test export block is missing');

const sandbox = {
  window: {},
  TextEncoder,
  Uint8Array,
  DataView,
  ArrayBuffer,
  console
};

vm.createContext(sandbox);
vm.runInContext(match[1], sandbox, { filename: 'external-monitor-protocol.js' });

const protocol = sandbox.window.ExternalIMUMonitorProtocol;
assert(protocol, 'protocol helper was not exported');

const ascii = new TextEncoder().encode('123456789');
assert.strictEqual(protocol.crc16Ccitt(ascii), 0x29B1);

const zeroCommand = protocol.encodeNativeCommand(protocol.commands.ZERO_CAL);
assert.deepStrictEqual(Array.from(zeroCommand.slice(0, 5)), [
  0xA5, 0x5A, 0x02, 0x01, 0x10
]);

const nativePayload = new Uint8Array(20);
const nativeView = new DataView(nativePayload.buffer);
nativeView.setUint32(0, 1234, true);
nativeView.setInt32(4, -250, true);
nativeView.setInt32(8, 90000, true);
nativeView.setInt32(12, -90000, true);
nativePayload[16] = 1;
nativePayload[17] = 3;
nativePayload[18] = 2;
nativePayload[19] = 3;

const nativeFrame = protocol.encodeNativeFrame(
  protocol.frameTypes.DATA,
  protocol.dataIds.IMU_STATUS,
  nativePayload
);

const parser = protocol.createMixedParser();
let parsedNative = null;
for (const byte of nativeFrame) {
  const frame = parser.pushByte(byte);
  if (frame?.kind === 'native-data') parsedNative = frame;
}

assert(parsedNative, 'native data frame was not parsed');
assert.strictEqual(parsedNative.timeMs, 1234);
assert.strictEqual(parsedNative.angularRateDps, -0.25);
assert.strictEqual(parsedNative.angleDeg, 90);
assert.strictEqual(parsedNative.normalizedAngleDeg, -90);
assert.strictEqual(parsedNative.imuState, 1);
assert.strictEqual(parsedNative.appState, 3);
assert.strictEqual(parsedNative.calResult, 2);
assert.strictEqual(parsedNative.flags, 3);

const jy901Angle = new Uint8Array(11);
jy901Angle[0] = 0x55;
jy901Angle[1] = 0x53;
jy901Angle[6] = 0x00;
jy901Angle[7] = 0x40;
for (let index = 0; index < 10; index += 1) {
  jy901Angle[10] = (jy901Angle[10] + jy901Angle[index]) & 0xFF;
}

let parsedJy901 = null;
for (const byte of jy901Angle) {
  const frame = parser.pushByte(byte);
  if (frame?.kind === 'jy901-angle') parsedJy901 = frame;
}

assert(parsedJy901, 'JY901 angle frame was not parsed');
assert.strictEqual(parsedJy901.yawDeg, 90);
