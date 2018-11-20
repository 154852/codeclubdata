const codeElement = document.querySelector('textarea');

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

if (window.location.href.split('#')[1].indexOf('invert') != -1) {
    document.body.style.filter = 'invert(1)';
}

if (window.location.href.split('#')[1].indexOf('hide') != -1) {
    const options = {
        'HTML (Taken from the <a href="https://google.com">google.com</a> home page)': 'google.html',
        'JavaScript (Taken from <a href="https://jquery.com/">jQuery</a>, a popular JS library)': 'jquery.js',
        'C++ (Taken from the <a href="https://github.com/ApolloAuto/apollo">Apollo</a> driverless car project)': 'apollo.cpp',
        'Java (Taken from my own game)': 'cosmoria.java',
        'Swift (Taken from the app <a href="https://github.com/Mortennn/Dozer/">Dozer</a> and some <a href="https://github.com/airbnb/">AirBNB</a> libraries)': 'dozerairbnb.swift',
        'Objective-C (Taken from a <a href="https://github.com/WhatsApp/stickers/">whatsapp</a> stickers engine)': 'whatsapp.m',
        'Shell (Taken from <a href="https://github.com/creationix/nvm/">NVM</a>, a <a href="https://nodejs.org/">Node</a> package manager)': 'nvm.sh',
        'Python (Taken from <a href="https://github.com/nasa/MLMCPy">MLMCPy</a>, a python library maintained by <a href="https://github.com/nasa/">NASA</a>)': 'nasa.pytext',
        'Python (Taken from <a href="https://github.com/spotify/chartify/">Chartify</a>, a library maintained by <a href="https://github.com/spotify/">Spotify</>)': 'spotify.pytext',
        'Ruby (Taken from <a href="https://github.com/socketry/falcon/">Falcon</a>, a web server application)': 'falcon.rb',
        'PHP (Taken from the <a href="https://github.com/tgalopin/html-sanitizer">HTML Sanitizer</a> library)': 'html-sanitizer.php'
    };

    const key = Object.keys(options)[parseInt(Math.random() * Object.keys(options).length)];
    document.querySelector('.language').innerHTML = 'Background code: ' + key;

    fetchPage('code/' + options[key], function() {
        let i = 100;

        const text = this.responseText.replace(/\s*\/\/.*/g, '').replace(/\s*\*.*/g, '').replace(/\s*#.*/g, '').replace(/\n\s*\n/g, '\n');

        let next = text.slice(0, i);

        setInterval(async function() {
            codeElement.innerText = next;
            codeElement.scrollTop = codeElement.scrollHeight;

            i += 1;

            next = text.slice(0, i % text.length);
        }, 0);
    });
}

const leniency = 1000;
const week = 1000 * 60 * 60 * 24 * 7;

fetchPage('data/skipped.json', function() {
    const json = JSON.parse(this.responseText);

    for (var i = 0; i < json.skipped.length; i++) {
        const date = new Date(json.skipped[i]);

        date.setHours(12);
        date.setMinutes(55);
        date.setMilliseconds(1000);

        json.skipped[i] = date.getTime();
    }

    const now = new Date();
    const isActualDay = now.getDay() == 2 && now.getHours() < 14;

    let nextDate = new Date(now.getTime());
    nextDate.setDate((now.getDate() + ((7 + 2) - now.getDay()) % 7) + (now.getDay() == 2 && now.getHours() > 14? 1:0));
    nextDate.setHours(12);
    nextDate.setMinutes(55);
    nextDate.setSeconds(0);

    let iterations = 0;

    while (true) {
        let accepted = true;

        for (const skipped of json.skipped) {
            if (Math.abs(skipped - nextDate.getTime()) < leniency) {
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
        else document.querySelector('.skipped').innerHTML = 'That means that code club is next on ' + (isActualDay?  '<u>today</u>':'<u>this coming Tuesday</u>')
    } else {
        dateElement.innerHTML = 'Never, code club has ended...';
        document.querySelector('.date-small').innerHTML = 'Or it is only going to be on again in a billion years or so, but this is just a little bit unlikely'
    }

    document.querySelector('.main').children[0].setAttribute('style', '');

    setTimeout(() => setInterval(function() {
        dateElement.style.opacity = dateElement.style.opacity == '0'? '1':'0';
    }, 750), 500);
});

fetchPage('https://api.ipify.org', function() {
    document.querySelector('.ip').innerText = 'Your IP address: ' + this.responseText;
})
