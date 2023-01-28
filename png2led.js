const Jimp = require('jimp');
const fs = require('fs');
const path = require('path');
const args = process.argv.slice(2);
if (!(args.length == 1 && args[0].endsWith('.png'))) {
    console.error('usage: node . <input file in png format>');
    process.exit(1);
}
if (!fs.existsSync(path.resolve(args[0]))) {
    console.error('input file does not exist');
    process.exit(1);
}

Jimp.read(path.resolve(args[0]), (err, image) => {
    if (err) throw err;

    let data = [];
    for (let i = 0; i < image.bitmap.height; i++) {
        let tmp = [];
        for (let j = 0; j < image.bitmap.width; j++) {
            tmp.push([0, 0, 0]);
        }
        data.push(tmp);
    }

    image.scan(0, 0, image.bitmap.width, image.bitmap.height, (x, y) => {
        data[y][x][0] = Jimp.intToRGBA(image.getPixelColor(x, y)).r;
        data[y][x][1] = Jimp.intToRGBA(image.getPixelColor(x, y)).g;
        data[y][x][2] = Jimp.intToRGBA(image.getPixelColor(x, y)).b;
    });

    let code = 'public static short[][][] ' + args[0].replace('.png', '').split('.').join('') + ' = new short[][][]' + JSON.stringify(data).replaceAll('[', '{').replaceAll(']', '}') + ';';
    fs.writeFileSync(path.resolve(args[0].replace('.png', '.java')), code);
});
