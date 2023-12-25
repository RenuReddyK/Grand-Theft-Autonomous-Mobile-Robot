const char motor[] PROGMEM = R"===(
<!DOCTYPE html>
<html>

<body>

    <div style="text-align: center;">
        <button type="button" onclick="sendMode('1')">Wall Following</button>
        <br />
        <button type="button" onclick="sendMode('4')">Push Police Car</button>
        <br />
        <button type="button" onclick="sendMode('2')">Beacon Track </button>
        <br />
        <button type="button" onclick="sendMode('0')">Stop</button>
        <br />
        
    </div>

    <script>

        function sendMode(Mode) {
            var xhttp = new XMLHttpRequest();
            var str = "Mode?val=";
            var res = str.concat(Mode);
            xhttp.open("GET", res, true);
            xhttp.send();
        }
    </script>
</body>

</html>
)===";
