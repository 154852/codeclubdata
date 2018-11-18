const textarea = document.querySelector('textarea');

Date.prototype.format = function(string) {
    const regex = '[CHAR]+';

    const data = {
        'n': this.getMilliseconds(),
        's': this.getSeconds(),
        'm': this.getMinutes(),
        'h': this.getHours(),
        'd': this.getDate(),
        'M': this.getMonth() + 1,
        'y': this.getFullYear()
    };

    for (const key in data) {
        const part = new RegExp(regex.replace('CHAR', key), 'g').exec(string);

        if (part != null) {
            for (var i = 0; i < part.length; i++) {
                string = string.replace(
                    part[i],
                    ((string, length, fill, end) => {
                        var extra = '';
                        for (var i = 0; i < length - string.length; i++) extra += fill;

                        return end? string + extra:extra + string;
                    })(data[key] + '', part[i].length, '0')
                );
            }
        }
    }

    return string;
}

function fetchPage(path, callback) {
    const xhr = new XMLHttpRequest();
    xhr.onload = callback;
    xhr.open('GET', path);
    xhr.send();
}

fetchPage('code/jquery.js', function() {
    let i = 100;

    const text = this.responseText.replace(/\s*\/\/.*/g, '').replace(/\n\s*\n/g, '\n');

    textarea.innerText = text.slice(0, i);

    setInterval(async function() {
        textarea.innerText = textarea.value + text.charAt(i % text.length);
        textarea.scrollTop = textarea.scrollHeight;

        i += 1;
    }, 0);
});

const leniency = 1000 * 60 * 60 * 24 * 2;
const week = 1000 * 60 * 60 * 24 * 7;

fetchPage('data/skipped.json', function() {
    const json = JSON.parse(this.responseText);

    const now = new Date();

    let nextDate = new Date(now.getTime());
    nextDate.setDate(now.getDate() + ((7 + 2) - now.getDay()) % 7);
    nextDate.setHours(12);
    nextDate.setMinutes(55);
    nextDate.setSeconds(0);

    let iterations = 0;

    while (true) {
        let accepted = true;

        for (const skipped of json.skipped) {
            if (nextDate.getTime() - skipped < leniency) {
                nextDate = new Date(nextDate.getTime() + week);
                accepted = false;
                break;
            }
        }

        if (accepted) break;

        iterations++;
        if (iterations == 10000 || nextDate.getTime() > json.max) {
            nextDate = null;
            break;
        }
    }

    const dateElement = document.querySelector('.date');

    if (nextDate != null) {
        dateElement.innerHTML = nextDate.format('hh:mm dd/MM/yyyy');
        document.querySelector('.date-small').innerHTML = nextDate.toGMTString().split(' ').slice(0, 4).join(' ');

        if (iterations != 0) document.querySelector('.skipped').innerHTML = 'That means that '  + (iterations == 1? '<u>this coming week</u> is':('the following <u>' + iterations + '</u> weeks are')) + ' skipped.'
        else document.querySelector('.skipped').innerHTML = 'That means that code club is on <u>this coming week</u>'
    } else {
        dateElement.innerHTML = 'Never, code club has ended...';
        document.querySelector('.date-small').innerHTML = 'Or it is only going to be on again in a billion years or so, but this is just a little bit unlikely'
    }

    document.querySelector('.main').setAttribute('style', '');

    setTimeout(() => setInterval(function() {
        dateElement.style.opacity = dateElement.style.opacity == '0'? '1':'0';
    }, 750), 500);
});

fetchPage('https://api.ipify.org', function() {
    document.querySelector('.ip').innerText = 'Your IP address: ' + this.responseText;
})