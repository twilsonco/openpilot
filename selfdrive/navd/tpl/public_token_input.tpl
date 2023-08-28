<form name="setPkTokenForm" method="post">
    <fieldset class="uk-fieldset">
        <legend class="uk-legend">Enter your <b>Mapbox Public Token:</b></legend>
        <div style="padding: 5px; color: red; font-weight: bold;">{{msg}}</div>
        <div class="uk-margin">
            <input class="uk-input" type="text" name="pk_token_val" placeholder="pk.xxxxxxx...">
            <input class="uk-button uk-button-primary uk-width-1-1 uk-margin-small-bottom" type="submit" value="Submit">
        </div>
        <div style="text-align: center;">
            <a href="https://account.mapbox.com/access-tokens" target="_blank" id="howToGetLink">How do I get this?</a>
        </div>
    </fieldset>
</form>
